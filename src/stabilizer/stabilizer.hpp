#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include <vector>

namespace fixed_camera_stabilizer {

struct Motion {
    // Pose of the current frame relative to the fixed reference frame.
    double dx = 0.0;
    double dy = 0.0;
    double rotation_radians = 0.0;
    bool valid = false;
    int tracked_feature_count = 0;
    int inlier_count = 0;
    double confidence = 0.0;
    double processing_time_ms = 0.0;
};

struct Timing {
    double resize_ms = 0.0;
    double grayscale_ms = 0.0;
    double tile_matching_ms = 0.0;
    double filtering_ms = 0.0;
    double total_ms = 0.0;
};

struct StabilizerConfig {
    double processing_scale = 0.25;
    double outer_band_ratio = 0.30;
    int maximum_features = 250;
    int minimum_inliers = 12;
    double feature_quality = 0.01;
    double feature_minimum_distance = 5.0;
    double maximum_tracking_error = 30.0;
    double minimum_inlier_ratio = 0.40;
    double ransac_threshold_pixels = 2.0;
    double maximum_displacement_pixels = 30.0;
    double maximum_rotation_degrees = 3.0;
    double correction_strength = 1.0;
    double crop_ratio = 0.05;
};

class Stabilizer {
public:
    explicit Stabilizer(StabilizerConfig config = {});

    Motion process(const cv::Mat& frame);
    void reset();

    const Timing& last_timing() const;

private:
    StabilizerConfig config_;
    cv::Mat reference_grayscale_;
    std::vector<cv::Point2f> reference_points_;
    Timing last_timing_;
};

}  // namespace fixed_camera_stabilizer
