#include "stabilizer/stabilizer.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>

namespace {

constexpr double kCropRatio = 0.05;

cv::Mat apply_correction(const cv::Mat& frame, const fixed_camera_stabilizer::Motion& motion)
{
    cv::Mat corrected;
    const double cosine = std::cos(motion.rotation_radians);
    const double sine = std::sin(motion.rotation_radians);
    const cv::Mat camera_pose = (cv::Mat_<double>(2, 3) << cosine, -sine, motion.dx,
                                                           sine, cosine, motion.dy);
    cv::Mat correction;
    cv::invertAffineTransform(camera_pose, correction);
    cv::warpAffine(frame, corrected, correction, frame.size(), cv::INTER_LINEAR,
                   cv::BORDER_REPLICATE);

    const int crop_x = static_cast<int>(frame.cols * kCropRatio);
    const int crop_y = static_cast<int>(frame.rows * kCropRatio);
    const cv::Rect crop(crop_x, crop_y, frame.cols - (2 * crop_x), frame.rows - (2 * crop_y));

    cv::Mat output;
    cv::resize(corrected(crop), output, frame.size(), 0.0, 0.0, cv::INTER_LINEAR);
    return output;
}

}  // namespace

int main(int argc, char* argv[])
{
    if (argc != 3) {
        std::cerr << "Usage: video_test <input.mp4> <output.mp4>\n";
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
    const int input_fourcc = static_cast<int>(input.get(cv::CAP_PROP_FOURCC));
    const cv::Size frame_size(width, height);

    cv::VideoWriter output(argv[2], input_fourcc, fps, frame_size);
    if (!output.isOpened()) {
        output.open(argv[2], cv::VideoWriter::fourcc('m', 'p', '4', 'v'), fps, frame_size);
    }
    if (!output.isOpened()) {
        std::cerr << "Could not create output video: " << argv[2] << '\n';
        return 1;
    }

    fixed_camera_stabilizer::Stabilizer stabilizer;
    cv::Mat frame;
    int frame_number = 0;
    while (input.read(frame)) {
        const fixed_camera_stabilizer::Motion motion = stabilizer.process(frame);
        const auto transform_start = std::chrono::steady_clock::now();
        const cv::Mat processed = apply_correction(frame, motion);
        const double transform_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - transform_start).count();
        output.write(processed);

        std::cout << "frame=" << frame_number++
                  << " dx=" << std::fixed << std::setprecision(2) << motion.dx
                  << " dy=" << motion.dy
                  << " rotation_deg=" << motion.rotation_radians * 180.0 / CV_PI
                  << " valid=" << (motion.valid ? "true" : "false")
                  << " tracked=" << motion.tracked_feature_count
                  << " inliers=" << motion.inlier_count
                  << " confidence=" << motion.confidence
                  << " resize_ms=" << stabilizer.last_timing().resize_ms
                  << " grayscale_ms=" << stabilizer.last_timing().grayscale_ms
                  << " tile_matching_ms=" << stabilizer.last_timing().tile_matching_ms
                  << " filtering_ms=" << stabilizer.last_timing().filtering_ms
                  << " transform_ms=" << transform_ms
                  << " total_ms=" << motion.processing_time_ms << '\n';
    }

    return 0;
}
