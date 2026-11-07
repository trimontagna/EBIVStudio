#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

#include "ebiv/events.h"
#include "ebiv/motion_comp.h"
#include "ebiv/params.h"
#include "ebiv/sampling.h"

namespace ebiv {

// One evaluated sample. Positions in sensor coordinates (x right, y down), velocities in [px/ms].
struct Vector {
    float x = 0, y = 0;  // sample centre
    float vx = 0, vy = 0;
    float peak   = 0;    // objective value (motion compensation) or correlation peak
    float events = 0;    // events used
    bool valid   = false; // false: no estimate (no / too few events)
};

// Post-processing results, recomputed from the raw vectors whenever the post-processing settings change.
struct Validated {
    std::vector<float> vx, vy;      // after outlier replacement
    std::vector<uint8_t> outlier;   // 1: flagged
    int outliers = 0, replaced = 0;
};

struct Field {
    int64_t t0 = 0;          // start of the time step [us]
    std::vector<Vector> v;   // ny * nx, row-major
    Validated post;
};

struct Result {
    Params params;
    Grid grid;
    int width = 0, height = 0;
    std::vector<Field> fields; // one per time step
    double seconds = 0;
    uint64_t samples = 0;
    std::string source;
};

struct ProcessProgress {
    std::atomic<double> fraction{0.0};
    std::atomic<bool> cancel{false};
    std::atomic<int> step{0};
};

// Runs the selected method on every sample of the grid and every time step. Blocking: call from a worker thread.
Result run_ebiv(const EventSet &set, const Params &params, ProcessProgress *progress = nullptr);

// Everything needed to understand one sample (Inspector window).
struct SampleDetail {
    Method method = Method::MotionComp;
    Roi roi;
    int64_t t0 = 0;
    Vector result;

    // motion compensation
    std::vector<double> vx, vy;
    ObjectiveMap map;
    ObjectivePeak peak;
    Image iwe_zero, iwe_best;     // warped with zero / estimated velocity

    // correlation methods
    Image plane;                  // (2h - 1) x (2w - 1), centre = zero displacement
    Image image_a, image_b;       // corr_sum: projections of the two sub-volumes; PIV: interrogation windows
    int events_a = 0, events_b = 0, slices_used = 0;
    Interval window_a, window_b;
};

SampleDetail inspect_sample(const EventSet &set, const Params &params, int64_t t0, int sample_x, int sample_y);

} // namespace ebiv
