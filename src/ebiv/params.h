#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "ebiv/events.h"

namespace ebiv {

enum class Method { MotionComp, CorrSum, Piv, Count };
const char *to_string(Method method);       // "motion_comp", "corr_sum", "piv"
const char *display_name(Method method);

// Objective ("reward") functions of the motion-compensation scheme (Stoffregen & Kleeman, CVPR 2019).
enum class Objective { Variance, SumSq, SumExp, R1, Isoa, Moa, Count };
const char *to_string(Objective objective);  // cfg spelling: "var", "SumSq", ...
bool parse_objective(const std::string &text, Objective &objective);

// Projection of the warped events onto the pixel grid. BilinearPython reproduces the weighting of
// ebiv_utils.sumWarpedEvents() (fx*fy on the floor pixel), for comparison with earlier Python results.
enum class Interpolation { Nearest, Bilinear, BilinearPython, Count };
const char *to_string(Interpolation interpolation);
bool parse_interpolation(const std::string &text, Interpolation &interpolation);

enum class PseudoMode { Count, Binary, Signed, TimeSurface, Num };
const char *to_string(PseudoMode mode);

struct Roi {
    int x = 0, y = 0, w = 0, h = 0;
};

// All processing parameters. Field names follow ebiv_params.cfg; times in [us] relative to the first event,
// velocities in [px/ms].
struct Params {
    Method method = Method::MotionComp;

    // [DEFAULT] time sampling
    bool single_time_step = true;
    int64_t t_start  = 10000;
    int64_t t_end    = 100000;
    int64_t t_step   = 5000;
    int64_t t_sample = 10000;
    // [DEFAULT] spatial sampling
    int sample_w = 40, sample_h = 40;
    int step_x = 20, step_y = 20;
    Polarity polarity   = Polarity::Positive;
    bool gauss_peak_fit = true;

    // [MOTION_COMP]
    double vx_min = -2, vx_max = 5, vy_min = -3, vy_max = 3;
    double res_x = 0.25, res_y = 0.25;
    Objective objective         = Objective::Variance;
    Interpolation interpolation = Interpolation::Bilinear;

    // [CORR_SUM]
    int64_t t_sep = 3000;
    int nt        = 20;

    // [PIV] pseudo-image pairs: frame A at t - delay/2, frame B at t + delay/2, each `piv_exposure` long
    int64_t piv_delay    = 2000;
    int64_t piv_exposure = 1000;
    PseudoMode piv_image = PseudoMode::Binary;
    // Pulsed illumination: frames start at pulse_offset + k * period, last one period, and are
    // `piv_pulse_separation` periods apart.
    bool piv_pulse_lock      = false;
    double pulse_freq_hz     = 200;
    int64_t pulse_offset_us  = 0;
    int piv_pulse_separation = 1;

    // [EBIV_STUDIO] extras
    bool roi_enabled = false;
    Roi roi;
    double magnification = 0; // [px/mm], 0: pixel units only
    int threads          = 0; // 0: all cores
};

bool load_cfg(const std::filesystem::path &path, Params &params, std::string *error);
bool save_cfg(const std::filesystem::path &path, const Params &params, std::string *error);

// np.arange(min, max + res, res)
std::vector<double> velocity_candidates(double vmin, double vmax, double res);
// Start times of the evaluated time steps.
std::vector<int64_t> time_steps(const Params &params);
// Time over which the displacement is measured [us] (sample duration, time offset or PIV delay).
double displacement_time_us(const Params &params);

struct Interval {
    int64_t t1 = 0, t2 = 0; // [t1, t2)
};
// PIV frames A / B for the time step starting at t0.
void piv_frames(const Params &params, int64_t t0, Interval &a, Interval &b);

struct Grid {
    int nx = 0, ny = 0;
    int x0 = 0, y0 = 0; // top-left of the first sample
    int dx = 1, dy = 1;
};
// Sampling grid (Python: range(0, imgW - sampleW + 1, sampleStepX), restricted to the ROI when enabled).
Grid sampling_grid(const Params &params, int width, int height);

} // namespace ebiv
