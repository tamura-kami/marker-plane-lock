#include "stabilizer/stabilizer.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>
#include <vector>

namespace fixed_camera_stabilizer {

namespace {

using Clock = std::chrono::steady_clock;

double elapsed_ms(const Clock::time_point& start)
{
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

cv::Mat outer_mask(const cv::Size& size, double band_ratio)
{
    cv::Mat mask(size, CV_8U, cv::Scalar(255));
    const int inset_x = std::clamp(static_cast<int>(size.width * band_ratio), 1,
                                   std::max(1, size.width / 2 - 1));
    const int inset_y = std::clamp(static_cast<int>(size.height * band_ratio), 1,
                                   std::max(1, size.height / 2 - 1));
    const cv::Rect center(inset_x, inset_y, size.width - 2 * inset_x,
                          size.height - 2 * inset_y);
    if (center.width > 0 && center.height > 0) {
        mask(center).setTo(0);
    }
    return mask;
}

}  // namespace

Stabilizer::Stabilizer(StabilizerConfig config)
    : config_(std::move(config))
{
}

Motion Stabilizer::process(const cv::Mat& frame)
{
    const auto total_start = Clock::now();
    last_timing_ = {};
    Motion motion;

    if (frame.empty() || config_.processing_scale <= 0.0 || config_.processing_scale > 1.0 ||
        config_.outer_band_ratio <= 0.0 || config_.outer_band_ratio >= 0.5) {
        motion.processing_time_ms = elapsed_ms(total_start);
        last_timing_.total_ms = motion.processing_time_ms;
        return motion;
    }

    const auto resize_start = Clock::now();
    cv::Mat scaled;
    cv::resize(frame, scaled, cv::Size(), config_.processing_scale, config_.processing_scale,
               cv::INTER_AREA);
    last_timing_.resize_ms = elapsed_ms(resize_start);

    const auto grayscale_start = Clock::now();
    cv::Mat grayscale;
    if (scaled.channels() == 1) {
        grayscale = scaled;
    } else if (scaled.channels() == 3) {
        cv::cvtColor(scaled, grayscale, cv::COLOR_BGR2GRAY);
    } else if (scaled.channels() == 4) {
        cv::cvtColor(scaled, grayscale, cv::COLOR_BGRA2GRAY);
    } else {
        motion.processing_time_ms = elapsed_ms(total_start);
        last_timing_.grayscale_ms = elapsed_ms(grayscale_start);
        last_timing_.total_ms = motion.processing_time_ms;
        return motion;
    }
    last_timing_.grayscale_ms = elapsed_ms(grayscale_start);

    // The first usable frame is the fixed camera pose. Keep it as an anchor instead
    // of integrating frame-to-frame measurements, which otherwise drift over time.
    if (reference_grayscale_.empty()) {
        reference_grayscale_ = grayscale.clone();
        cv::goodFeaturesToTrack(reference_grayscale_, reference_points_, config_.maximum_features,
                                config_.feature_quality, config_.feature_minimum_distance,
                                outer_mask(reference_grayscale_.size(), config_.outer_band_ratio));
        motion.tracked_feature_count = static_cast<int>(reference_points_.size());
        motion.processing_time_ms = elapsed_ms(total_start);
        last_timing_.total_ms = motion.processing_time_ms;
        return motion;
    }

    const auto matching_start = Clock::now();
    if (reference_points_.size() < static_cast<size_t>(config_.minimum_inliers)) {
        motion.processing_time_ms = elapsed_ms(total_start);
        last_timing_.tile_matching_ms = elapsed_ms(matching_start);
        last_timing_.total_ms = motion.processing_time_ms;
        return motion;
    }

    std::vector<cv::Point2f> current_points;
    std::vector<unsigned char> tracking_status;
    std::vector<float> tracking_error;
    cv::calcOpticalFlowPyrLK(reference_grayscale_, grayscale, reference_points_, current_points,
                             tracking_status, tracking_error, cv::Size(21, 21), 3);

    std::vector<cv::Point2f> matched_reference;
    std::vector<cv::Point2f> matched_current;
    matched_reference.reserve(reference_points_.size());
    matched_current.reserve(reference_points_.size());
    for (size_t i = 0; i < tracking_status.size(); ++i) {
        if (!tracking_status[i] || !std::isfinite(current_points[i].x) ||
            !std::isfinite(current_points[i].y) ||
            tracking_error[i] > config_.maximum_tracking_error) {
            continue;
        }
        const double displacement = cv::norm(current_points[i] - reference_points_[i]) /
                                    config_.processing_scale;
        if (displacement <= config_.maximum_displacement_pixels) {
            matched_reference.push_back(reference_points_[i]);
            matched_current.push_back(current_points[i]);
        }
    }
    motion.tracked_feature_count = static_cast<int>(matched_reference.size());

    if (matched_reference.size() >= static_cast<size_t>(config_.minimum_inliers)) {
        cv::Mat inlier_mask;
        const cv::Mat affine = cv::estimateAffinePartial2D(
            matched_reference, matched_current, inlier_mask, cv::RANSAC,
            config_.ransac_threshold_pixels * config_.processing_scale, 2000, 0.99, 10);

        if (!affine.empty()) {
            motion.inlier_count = cv::countNonZero(inlier_mask);
            motion.confidence = static_cast<double>(motion.inlier_count) /
                                static_cast<double>(matched_reference.size());

            const double a = affine.at<double>(0, 0);
            const double c = affine.at<double>(1, 0);
            const double angle = std::atan2(c, a);
            const double dx = affine.at<double>(0, 2) / config_.processing_scale;
            const double dy = affine.at<double>(1, 2) / config_.processing_scale;
            const double max_angle = config_.maximum_rotation_degrees * CV_PI / 180.0;

            motion.valid = motion.inlier_count >= config_.minimum_inliers &&
                           motion.confidence >= config_.minimum_inlier_ratio &&
                           std::hypot(dx, dy) <= config_.maximum_displacement_pixels &&
                           std::abs(angle) <= max_angle;
            if (motion.valid) {
                // A fixed camera has no intentional trajectory to follow: apply the
                // full anchor-relative pose so vibration is not absorbed by a filter.
                motion.dx = dx * config_.correction_strength;
                motion.dy = dy * config_.correction_strength;
                motion.rotation_radians = angle * config_.correction_strength;
            }
        }
    }
    last_timing_.tile_matching_ms = elapsed_ms(matching_start);

    const auto filtering_start = Clock::now();
    const double maximum_x = frame.cols * config_.crop_ratio;
    const double maximum_y = frame.rows * config_.crop_ratio;
    motion.dx = std::clamp(motion.dx, -maximum_x, maximum_x);
    motion.dy = std::clamp(motion.dy, -maximum_y, maximum_y);
    last_timing_.filtering_ms = elapsed_ms(filtering_start);

    motion.processing_time_ms = elapsed_ms(total_start);
    last_timing_.total_ms = motion.processing_time_ms;
    return motion;
}

void Stabilizer::reset()
{
    reference_grayscale_.release();
    reference_points_.clear();
    last_timing_ = {};
}

const Timing& Stabilizer::last_timing() const
{
    return last_timing_;
}

}  // namespace fixed_camera_stabilizer
