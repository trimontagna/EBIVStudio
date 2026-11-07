#pragma once

#include <vector>

#include "ebiv/processor.h"

namespace ebiv {

struct PostParams {
    bool enabled          = true;
    // Normalized median test on vx, vy and |v| (velmap_utils.DetectOutliers): a vector is an outlier when at
    // least two of the three tests fail. 0 disables the test.
    double median_threshold = 3.5;
    double noise_floor      = 0.05;  // [px/ms]
    double max_magnitude    = 0;     // [px/ms], 0: off
    double min_events       = 0;     // samples with fewer events are outliers
    double min_peak         = 0;     // samples with a lower peak value are outliers
    bool flag_empty         = true;  // samples without estimate are outliers
    bool replace            = true;  // replace outliers by the mean of their valid neighbours
};

// Fills field.post from field.v.
void validate(Field &field, const Grid &grid, const PostParams &params);

// Normalized median test of one scalar field (velmap_utils.NormalizedMedianFilter): 1 where it fails.
std::vector<uint8_t> normalized_median_test(const std::vector<float> &values, int nx, int ny, double threshold,
                                            double noise_floor);

enum class Scalar { Magnitude, Vx, Vy, Vorticity, Divergence, Peak, Events, Count };
const char *to_string(Scalar scalar);

// Scalar map of a field (post-processed velocities). Velocities [px/ms], vorticity / divergence [1/ms] in sensor
// orientation: vorticity = du/dy - dv/dx, which is positive for counter-clockwise rotation as seen with y up.
std::vector<float> scalar_field(const Field &field, const Grid &grid, Scalar scalar);

} // namespace ebiv
