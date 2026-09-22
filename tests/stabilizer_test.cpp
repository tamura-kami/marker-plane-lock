#include "stabilizer/stabilizer.hpp"

#include <opencv2/imgproc.hpp>

#include <cmath>
#include <iostream>

namespace {

cv::Mat translated(const cv::Mat& source, double dx, double dy)
{
    cv::Mat result;
    const cv::Mat transform = (cv::Mat_<double>(2, 3) << 1.0, 0.0, dx,
                                                       0.0, 1.0, dy);
    cv::warpAffine(source, result, transform, source.size(), cv::INTER_LINEAR,
                   cv::BORDER_REFLECT);
    return result;
}

cv::Mat transformed(const cv::Mat& source, double dx, double dy, double angle_degrees)
{
    const double angle = angle_degrees * CV_PI / 180.0;
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    cv::Mat result;
    const cv::Mat transform = (cv::Mat_<double>(2, 3) << cosine, -sine, dx,
                                                       sine, cosine, dy);
    cv::warpAffine(source, result, transform, source.size(), cv::INTER_LINEAR,
                   cv::BORDER_REFLECT);
    return result;
}

cv::Mat detailed_frame()
{
    cv::Mat frame(480, 640, CV_8UC3, cv::Scalar(235, 235, 235));
    for (int x = 0; x < frame.cols; x += 24) {
        cv::line(frame, {x, 0}, {x, frame.rows - 1}, cv::Scalar(60, 90, 130), 2);
    }
    for (int y = 0; y < frame.rows; y += 20) {
        cv::line(frame, {0, y}, {frame.cols - 1, y}, cv::Scalar(130, 70, 40), 2);
    }
    cv::putText(frame, "fixed background", {18, 50}, cv::FONT_HERSHEY_SIMPLEX,
                1.0, cv::Scalar(0, 0, 0), 2);
    return frame;
}

bool expect(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << message << '\n';
    }
    return condition;
}

}  // namespace

int main()
{
    bool ok = true;

    fixed_camera_stabilizer::Stabilizer flat_stabilizer;
    const cv::Mat flat(480, 640, CV_8UC3, cv::Scalar(220, 220, 220));
    flat_stabilizer.process(flat);
    const auto flat_motion = flat_stabilizer.process(flat);
    ok &= expect(!flat_motion.valid, "A featureless frame must not produce motion");
    ok &= expect(flat_motion.tracked_feature_count == 0,
                 "Featureless outer tiles must be ignored automatically");

    fixed_camera_stabilizer::Stabilizer moving_stabilizer;
    const cv::Mat background = detailed_frame();
    moving_stabilizer.process(background);
    const auto global_motion = moving_stabilizer.process(translated(background, 4.0, -3.0));
    ok &= expect(global_motion.valid, "Consistent motion on the outer edges must be accepted");
    ok &= expect(global_motion.inlier_count >= 12,
                 "Global motion must reach feature consensus");
    ok &= expect(std::abs(global_motion.dx - 4.0) < 1.0 &&
                     std::abs(global_motion.dy + 3.0) < 1.0,
                 "Anchor-relative translation must be recovered");

    fixed_camera_stabilizer::Stabilizer rotation_stabilizer;
    rotation_stabilizer.process(background);
    const auto rotated_motion = rotation_stabilizer.process(
        transformed(background, 3.0, -2.0, 1.25));
    ok &= expect(rotated_motion.valid, "Small camera rotation must be accepted");
    ok &= expect(std::abs(rotated_motion.rotation_radians * 180.0 / CV_PI - 1.25) < 0.35,
                 "Small camera rotation must be recovered");

    const auto returned_motion = rotation_stabilizer.process(background);
    ok &= expect(returned_motion.valid && std::hypot(returned_motion.dx, returned_motion.dy) < 0.5 &&
                     std::abs(returned_motion.rotation_radians) < 0.002,
                 "Returning to the anchor must produce no residual drift");

    fixed_camera_stabilizer::Stabilizer local_stabilizer;
    local_stabilizer.process(background);
    cv::Mat local_change = background.clone();
    cv::rectangle(local_change, {250, 170, 140, 140}, cv::Scalar(0, 0, 0), cv::FILLED);
    const auto local_motion = local_stabilizer.process(local_change);
    ok &= expect(local_motion.valid,
                 "A central moving object must not hide stable outer-edge consensus");

    return ok ? 0 : 1;
}
