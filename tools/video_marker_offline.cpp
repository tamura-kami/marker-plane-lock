#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace {

using MarkerCorners = std::array<cv::Point2f, 4>;
using Detection = std::map<int, MarkerCorners>;

Detection detect_markers(const cv::Mat& frame,
                         const cv::Ptr<cv::aruco::Dictionary>& dictionary,
                         const cv::Ptr<cv::aruco::DetectorParameters>& parameters)
{
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<std::vector<cv::Point2f>> rejected;
    cv::aruco::detectMarkers(frame, dictionary, corners, ids, parameters, rejected);

    Detection result;
    for (size_t i = 0; i < ids.size(); ++i) {
        if (ids[i] < 0 || ids[i] > 3 || corners[i].size() != 4) {
            continue;
        }
        MarkerCorners marker;
        std::copy_n(corners[i].begin(), 4, marker.begin());
        result[ids[i]] = marker;
    }
    return result;
}

double reference_score(const Detection& detection)
{
    double perimeter_sum = 0.0;
    for (const auto& [id, corners] : detection) {
        (void)id;
        for (int i = 0; i < 4; ++i) {
            perimeter_sum += cv::norm(corners[i] - corners[(i + 1) % 4]);
        }
    }
    return static_cast<double>(detection.size()) * 1.0e6 + perimeter_sum;
}

cv::Mat identity_homography()
{
    return cv::Mat::eye(3, 3, CV_64F);
}

cv::Mat estimate_to_reference(const Detection& current, const Detection& reference)
{
    std::vector<cv::Point2f> source;
    std::vector<cv::Point2f> destination;
    int shared_markers = 0;
    for (const auto& [id, current_corners] : current) {
        const auto found = reference.find(id);
        if (found == reference.end()) {
            continue;
        }
        ++shared_markers;
        if (current.size() >= 2) {
            cv::Point2f current_center;
            cv::Point2f reference_center;
            for (int corner = 0; corner < 4; ++corner) {
                current_center += current_corners[corner] * 0.25F;
                reference_center += found->second[corner] * 0.25F;
            }
            source.push_back(current_center);
            destination.push_back(reference_center);
        } else {
            for (int corner = 0; corner < 4; ++corner) {
                source.push_back(current_corners[corner]);
                destination.push_back(found->second[corner]);
            }
        }
    }

    if (shared_markers >= 2) {
        cv::Mat inliers;
        // A fixed overhead camera needs a stable similarity transform, not an
        // eight-degree-of-freedom homography that magnifies subpixel corner noise.
        const cv::Mat affine = cv::estimateAffinePartial2D(
            source, destination, inliers, cv::RANSAC, 1.5, 2000, 0.995, 10);
        if (!affine.empty() && cv::countNonZero(inliers) >= 2) {
            cv::Mat homography = identity_homography();
            affine.copyTo(homography(cv::Rect(0, 0, 3, 2)));
            return homography;
        }
    } else if (shared_markers == 1) {
        // One visible tag still supplies rotation, scale and translation. Restricting
        // this fallback to a similarity transform avoids unstable projective warps.
        cv::Mat inliers;
        const cv::Mat affine = cv::estimateAffinePartial2D(source, destination, inliers,
                                                           cv::LMEDS);
        if (!affine.empty()) {
            cv::Mat homography = identity_homography();
            affine.copyTo(homography(cv::Rect(0, 0, 3, 2)));
            return homography;
        }
    }
    return {};
}

cv::Point2d transform_point(const cv::Mat& transform, const cv::Point2d& point)
{
    const double denominator = transform.at<double>(2, 0) * point.x +
                               transform.at<double>(2, 1) * point.y +
                               transform.at<double>(2, 2);
    return {
        (transform.at<double>(0, 0) * point.x + transform.at<double>(0, 1) * point.y +
         transform.at<double>(0, 2)) / denominator,
        (transform.at<double>(1, 0) * point.x + transform.at<double>(1, 1) * point.y +
         transform.at<double>(1, 2)) / denominator,
    };
}

void suppress_subpixel_jitter(std::vector<cv::Mat>& homographies, const cv::Size& frame_size)
{
    if (homographies.empty()) {
        return;
    }
    constexpr double kMaximumInvisibleChangePixels = 0.4;
    const std::array<cv::Point2d, 5> probes = {{
        {0.0, 0.0},
        {static_cast<double>(frame_size.width - 1), 0.0},
        {0.0, static_cast<double>(frame_size.height - 1)},
        {static_cast<double>(frame_size.width - 1),
         static_cast<double>(frame_size.height - 1)},
        {frame_size.width * 0.5, frame_size.height * 0.5},
    }};

    cv::Mat accepted = homographies.front().clone();
    for (cv::Mat& homography : homographies) {
        double maximum_change = 0.0;
        for (const cv::Point2d& probe : probes) {
            maximum_change = std::max(
                maximum_change,
                cv::norm(transform_point(homography, probe) -
                         transform_point(accepted, probe)));
        }
        if (maximum_change <= kMaximumInvisibleChangePixels) {
            homography = accepted.clone();
        } else {
            accepted = homography.clone();
        }
    }
}

void interpolate_missing(std::vector<cv::Mat>& homographies)
{
    size_t first_valid = 0;
    while (first_valid < homographies.size() && homographies[first_valid].empty()) {
        ++first_valid;
    }
    if (first_valid == homographies.size()) {
        return;
    }
    for (size_t i = 0; i < first_valid; ++i) {
        homographies[i] = homographies[first_valid].clone();
    }

    size_t index = first_valid + 1;
    while (index < homographies.size()) {
        if (!homographies[index].empty()) {
            ++index;
            continue;
        }
        const size_t gap_begin = index;
        while (index < homographies.size() && homographies[index].empty()) {
            ++index;
        }
        const size_t left = gap_begin - 1;
        if (index == homographies.size()) {
            for (size_t i = gap_begin; i < index; ++i) {
                homographies[i] = homographies[left].clone();
            }
            break;
        }
        const size_t right = index;
        for (size_t i = gap_begin; i < right; ++i) {
            const double alpha = static_cast<double>(i - left) /
                                 static_cast<double>(right - left);
            homographies[i] = (1.0 - alpha) * homographies[left] +
                              alpha * homographies[right];
            homographies[i] /= homographies[i].at<double>(2, 2);
        }
    }
}

cv::Rect largest_valid_rectangle(const cv::Mat& mask)
{
    std::vector<int> heights(mask.cols, 0);
    cv::Rect best;
    for (int y = 0; y < mask.rows; ++y) {
        const auto* row = mask.ptr<unsigned char>(y);
        for (int x = 0; x < mask.cols; ++x) {
            heights[x] = row[x] ? heights[x] + 1 : 0;
        }
        std::vector<int> stack;
        for (int x = 0; x <= mask.cols; ++x) {
            const int height = x == mask.cols ? 0 : heights[x];
            while (!stack.empty() && heights[stack.back()] > height) {
                const int top = stack.back();
                stack.pop_back();
                const int left = stack.empty() ? 0 : stack.back() + 1;
                const cv::Rect candidate(left, y - heights[top] + 1,
                                         x - left, heights[top]);
                if (candidate.area() > best.area()) {
                    best = candidate;
                }
            }
            if (x < mask.cols) {
                stack.push_back(x);
            }
        }
    }
    if (best.width % 2) --best.width;
    if (best.height % 2) --best.height;
    return best;
}

bool open_writer(cv::VideoWriter& writer, const std::string& path, double fps,
                 const cv::Size& size, int preferred_fourcc)
{
    writer.open(path, preferred_fourcc, fps, size);
    if (!writer.isOpened()) {
        writer.open(path, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), fps, size);
    }
    return writer.isOpened();
}

}  // namespace

int main(int argc, char* argv[])
{
    if (argc != 3) {
        std::cerr << "Usage: video_marker_offline <input.mp4> <output.mp4>\n";
        return 1;
    }

    cv::VideoCapture input(argv[1]);
    if (!input.isOpened()) {
        std::cerr << "Could not open input video: " << argv[1] << '\n';
        return 1;
    }
    const int width = static_cast<int>(input.get(cv::CAP_PROP_FRAME_WIDTH));
    const int height = static_cast<int>(input.get(cv::CAP_PROP_FRAME_HEIGHT));
    const double fps = input.get(cv::CAP_PROP_FPS);
    const int fourcc = static_cast<int>(input.get(cv::CAP_PROP_FOURCC));
    const cv::Size frame_size(width, height);

    const auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    const auto parameters = cv::aruco::DetectorParameters::create();
    parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    parameters->cornerRefinementWinSize = 5;

    std::vector<Detection> detections;
    cv::Mat frame;
    while (input.read(frame)) {
        detections.push_back(detect_markers(frame, dictionary, parameters));
    }
    if (detections.empty()) {
        std::cerr << "Input contains no frames\n";
        return 1;
    }

    size_t reference_index = 0;
    double best_score = -1.0;
    for (size_t i = 0; i < detections.size(); ++i) {
        const double score = reference_score(detections[i]);
        if (score > best_score) {
            best_score = score;
            reference_index = i;
        }
    }
    const Detection& reference = detections[reference_index];
    if (reference.size() < 2) {
        std::cerr << "At least two IDs from DICT_4X4_50 IDs 0-3 must be visible in one frame\n";
        return 1;
    }

    std::vector<cv::Mat> homographies(detections.size());
    size_t direct_frames = 0;
    std::array<size_t, 5> marker_histogram{};
    for (size_t i = 0; i < detections.size(); ++i) {
        marker_histogram[std::min<size_t>(4, detections[i].size())]++;
        homographies[i] = estimate_to_reference(detections[i], reference);
        if (!homographies[i].empty()) {
            ++direct_frames;
        }
    }
    interpolate_missing(homographies);
    if (std::any_of(homographies.begin(), homographies.end(),
                    [](const cv::Mat& h) { return h.empty(); })) {
        std::cerr << "Could not estimate a transform for the video\n";
        return 1;
    }
    suppress_subpixel_jitter(homographies, frame_size);

    cv::Mat common_valid(frame_size, CV_8U, cv::Scalar(255));
    const cv::Mat source_valid(frame_size, CV_8U, cv::Scalar(255));
    cv::Mat warped_mask;
    for (const cv::Mat& homography : homographies) {
        cv::warpPerspective(source_valid, warped_mask, homography, frame_size,
                            cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));
        cv::bitwise_and(common_valid, warped_mask, common_valid);
    }
    const cv::Rect crop = largest_valid_rectangle(common_valid);
    if (crop.width < 16 || crop.height < 16) {
        std::cerr << "No common valid output area remains\n";
        return 1;
    }

    input.release();
    input.open(argv[1]);
    cv::VideoWriter output;
    if (!open_writer(output, argv[2], fps, crop.size(), fourcc)) {
        std::cerr << "Could not create output video: " << argv[2] << '\n';
        return 1;
    }

    cv::Mat stabilized;
    size_t frame_index = 0;
    while (frame_index < homographies.size() && input.read(frame)) {
        cv::warpPerspective(frame, stabilized, homographies[frame_index], frame_size,
                            cv::INTER_LANCZOS4, cv::BORDER_CONSTANT);
        output.write(stabilized(crop));
        ++frame_index;
    }

    std::cout << "frames=" << detections.size()
              << " direct=" << direct_frames
              << " interpolated=" << detections.size() - direct_frames
              << " reference_frame=" << reference_index
              << " reference_markers=" << reference.size()
              << " seen_0=" << marker_histogram[0]
              << " seen_1=" << marker_histogram[1]
              << " seen_2=" << marker_histogram[2]
              << " seen_3=" << marker_histogram[3]
              << " seen_4=" << marker_histogram[4]
              << " input=" << width << 'x' << height
              << " output=" << crop.width << 'x' << crop.height
              << " crop_left=" << crop.x
              << " crop_top=" << crop.y << '\n';
    return 0;
}
