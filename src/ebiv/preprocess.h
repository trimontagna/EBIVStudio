#pragma once

#include <atomic>
#include <string>

#include "core/filters.h"
#include "core/hot_pixels.h"
#include "ebiv/events.h"
#include "ebiv/params.h"

namespace ebiv {

struct PreprocessParams {
    // Hot pixels are detected on the loaded recording and removed (EventStudio's detector).
    bool hot_pixels = false;
    evs::HotPixelParams hot;

    // Software filters of EventStudio (activity, refractory, trail, polarity).
    evs::FilterConfig filters;

    // Keep only the events inside a rectangle / a time range.
    bool crop_roi = false;
    Roi roi;
    bool crop_time   = false;
    int64_t t_begin  = 0, t_end = 0; // [us]
};

struct PreprocessProgress {
    std::atomic<double> fraction{0.0};
    std::atomic<bool> cancel{false};
};

struct PreprocessReport {
    uint64_t in = 0, out = 0;
    size_t hot_pixels = 0;
    double seconds    = 0;
};

bool any_stage(const PreprocessParams &params);

// Runs the enabled stages over `in` and writes the kept events to `out` (same geometry, same time base).
bool preprocess(const EventSet &in, const PreprocessParams &params, EventSet &out, PreprocessReport &report,
                std::string *error, PreprocessProgress *progress = nullptr);

} // namespace ebiv
