#pragma once

#include <cstdint>
#include <vector>

#include "ebiv/events.h"

namespace ebiv {

// Mean event rate over one illumination period (EBI::MeanPulseHistogram / DetermineOffsetTime).
struct PulseHistogram {
    double freq_hz   = 0;
    double period_us = 0;
    int bin_us       = 0;
    int periods      = 0;          // periods actually covered by the data
    std::vector<double> rate;      // [events / us] per bin, referenced to t = 0 of the recording
    int64_t offset_us = 0;         // estimated start of the pulse: minimum of the histogram before its maximum
    double max_rate = 0, min_rate = 0;
    int max_bin = 0;
};

// Positive events of `periods` periods starting at period `start_period`.
PulseHistogram pulse_histogram(const EventSet &set, double freq_hz, int bin_us, int periods, int start_period,
                               Polarity polarity = Polarity::Positive);

} // namespace ebiv
