#include "stabilizer/stabilizer.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

using fixed_camera_stabilizer::Motion;

std::string shell_quote(const std::string& value)
{
    std::string quoted = "'";
    for (const char character : value) {
        if (character == '\'') {
            quoted += "'\\''";
        } else {
            quoted += character;
        }
    }
    quoted += "'";
    return quoted;
}

cv::Mat correction_matrix(const Motion& motion)
{
    const double cosine = std::cos(motion.rotation_radians);
    const double sine = std::sin(motion.rotation_radians);
    const cv::Mat camera_pose = (cv::Mat_<double>(2, 3) << cosine, -sine, motion.dx,
                                                           sine, cosine, motion.dy);
    cv::Mat correction;
    cv::invertAffineTransform(camera_pose, correction);
    return correction;
}

double median(std::vector<double> values)
{
    if (values.empty()) {
        return 0.0;
    }
    const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
    std::nth_element(values.begin(), middle, values.end());
    return *middle;
}

void interpolate_invalid(std::vector<Motion>& motions)
{
    if (motions.empty()) {
        return;
    }
    motions.front().valid = true;

    size_t index = 0;
    while (index < motions.size()) {
        if (motions[index].valid) {
            ++index;
            continue;
        }
        const size_t gap_begin = index;
        while (index < motions.size() && !motions[index].valid) {
            ++index;
        }
        const size_t left = gap_begin == 0 ? index : gap_begin - 1;
        const size_t right = index < motions.size() ? index : left;
        for (size_t i = gap_begin; i < index; ++i) {
            const double alpha = right == left ? 0.0
                                                : static_cast<double>(i - left) /
                                                      static_cast<double>(right - left);
            motions[i].dx = motions[left].dx + alpha * (motions[right].dx - motions[left].dx);
            motions[i].dy = motions[left].dy + alpha * (motions[right].dy - motions[left].dy);
            motions[i].rotation_radians = motions[left].rotation_radians +
                alpha * (motions[right].rotation_radians - motions[left].rotation_radians);
            motions[i].valid = true;
        }
    }
}

void replace_low_confidence_outliers(std::vector<Motion>& motions)
{
    const std::vector<Motion> original = motions;
    constexpr int radius = 2;
    for (size_t i = 0; i < motions.size(); ++i) {
        std::vector<double> x_values;
        std::vector<double> y_values;
        std::vector<double> angle_values;
        const size_t begin = i > radius ? i - radius : 0;
        const size_t end = std::min(motions.size(), i + radius + 1);
        for (size_t j = begin; j < end; ++j) {
            x_values.push_back(original[j].dx);
            y_values.push_back(original[j].dy);
            angle_values.push_back(original[j].rotation_radians);
        }
        const double median_x = median(std::move(x_values));
        const double median_y = median(std::move(y_values));
        const double median_angle = median(std::move(angle_values));
        const bool translation_outlier =
            std::hypot(original[i].dx - median_x, original[i].dy - median_y) > 3.0;
        const bool rotation_outlier =
            std::abs(original[i].rotation_radians - median_angle) > 0.35 * CV_PI / 180.0;
        if (original[i].confidence < 0.55 && (translation_outlier || rotation_outlier)) {
            motions[i].dx = median_x;
            motions[i].dy = median_y;
            motions[i].rotation_radians = median_angle;
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
            heights[x] = row[x] != 0 ? heights[x] + 1 : 0;
        }

        std::vector<int> stack;
        for (int x = 0; x <= mask.cols; ++x) {
            const int height = x == mask.cols ? 0 : heights[x];
            while (!stack.empty() && heights[stack.back()] > height) {
                const int top = stack.back();
                stack.pop_back();
                const int left = stack.empty() ? 0 : stack.back() + 1;
                const int width = x - left;
                const int area = heights[top] * width;
                if (area > best.area()) {
                    best = cv::Rect(left, y - heights[top] + 1, width, heights[top]);
                }
            }
            if (x < mask.cols) {
                stack.push_back(x);
            }
        }
    }
    return best;
}

cv::Rect even_sized(cv::Rect rectangle)
{
    if (rectangle.width % 2 != 0) {
        --rectangle.width;
    }
    if (rectangle.height % 2 != 0) {
        --rectangle.height;
    }
    return rectangle;
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
        std::cerr << "Usage: video_offline <input.mp4> <output.mp4>\n";
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

    fixed_camera_stabilizer::Stabilizer stabilizer;
    std::vector<Motion> motions;
    cv::Mat frame;
    while (input.read(frame)) {
        motions.push_back(stabilizer.process(frame));
    }
    if (motions.empty()) {
        std::cerr << "Input contains no video frames\n";
        return 1;
    }

    const size_t detected = static_cast<size_t>(std::count_if(
        motions.begin(), motions.end(), [](const Motion& motion) { return motion.valid; }));
    interpolate_invalid(motions);
    replace_low_confidence_outliers(motions);

    // Intersect the valid pixels from every corrected frame. This lets the offline
    // pass find the smallest safe crop instead of enlarging a fixed 5% crop.
    cv::Mat common_valid(frame_size, CV_8U, cv::Scalar(255));
    const cv::Mat source_valid(frame_size, CV_8U, cv::Scalar(255));
    cv::Mat corrected_valid;
    for (const Motion& motion : motions) {
        cv::warpAffine(source_valid, corrected_valid, correction_matrix(motion), frame_size,
                       cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));
        cv::bitwise_and(common_valid, corrected_valid, common_valid);
    }
    const cv::Rect crop = even_sized(largest_valid_rectangle(common_valid));
    if (crop.width < 16 || crop.height < 16) {
        std::cerr << "Could not find a common valid crop area\n";
        return 1;
    }

    input.set(cv::CAP_PROP_POS_FRAMES, 0);
    const std::string video_only_path = std::string(argv[2]) + ".video-only.mp4";
    cv::VideoWriter output;
    if (!open_writer(output, video_only_path, fps, crop.size(), input_fourcc)) {
        std::cerr << "Could not create output video: " << argv[2] << '\n';
        return 1;
    }

    size_t frame_number = 0;
    cv::Mat corrected;
    while (frame_number < motions.size() && input.read(frame)) {
        cv::warpAffine(frame, corrected, correction_matrix(motions[frame_number]), frame_size,
                       cv::INTER_LANCZOS4, cv::BORDER_CONSTANT);
        output.write(corrected(crop));
        ++frame_number;
    }
    output.release();

    const std::string mux_command = "ffmpeg -y -v error -i " +
        shell_quote(video_only_path) + " -i " + shell_quote(argv[1]) +
        " -map 0:v:0 -map 1:a? -map_metadata 1 -c:v copy -c:a aac -shortest "
        "-movflags +faststart " + shell_quote(argv[2]);
    if (std::system(mux_command.c_str()) != 0) {
        std::cerr << "Could not mux the original audio into output; intermediate video is at "
                  << video_only_path << "\n";
        return 1;
    }
    std::filesystem::remove(video_only_path);

    std::cout << "frames=" << motions.size()
              << " detected=" << detected
              << " interpolated=" << motions.size() - detected
              << " input=" << width << 'x' << height
              << " output=" << crop.width << 'x' << crop.height
              << " crop_left=" << crop.x
              << " crop_top=" << crop.y << '\n';
    return 0;
}
