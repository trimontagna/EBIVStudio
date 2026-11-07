#include "ebiv/pulse.h"

#include <algorithm>
#include <cmath>

namespace ebiv {

PulseHistogram pulse_histogram(const EventSet &set, double freq_hz, int bin_us, int periods, int start_period,
                               Polarity polarity) {
    PulseHistogram h;
    if (!(freq_hz > 0) || bin_us <= 0 || periods <= 0 || set.empty())
        return h;
    h.freq_hz   = freq_hz;
    h.period_us = 1e6 / freq_hz;
    h.bin_us    = bin_us;
    const int bins = std::max(1, static_cast<int>(h.period_us / bin_us));
    h.rate.assign(bins, 0.0);

    const int64_t t0 = static_cast<int64_t>(h.period_us * std::max(0, start_period));
    const int64_t t1 = std::min<int64_t>(t0 + static_cast<int64_t>(periods * h.period_us),
                                         int64_t(set.duration_us()) + 1);
    h.periods = std::max(1, static_cast<int>(std::ceil((t1 - t0) / h.period_us)));
    const auto [b, e] = set.time_range(t0, t1);
    for (size_t i = b; i < e; ++i) {
        const Event &ev = set.events[i];
        if (!matches(polarity, ev))
            continue;
        const double t   = ev.t;
        const double rem = std::floor(t - std::floor(t / h.period_us) * h.period_us);
        const int bin    = static_cast<int>(rem / bin_us);
        if (bin >= 0 && bin < bins)
            h.rate[bin] += 1;
    }
    for (double &r : h.rate)
        r /= static_cast<double>(h.periods) * bin_us;

    // DetermineOffsetTime: from the maximum, walk back one full period and keep the minimum
    h.max_bin  = static_cast<int>(std::max_element(h.rate.begin(), h.rate.end()) - h.rate.begin());
    h.max_rate = h.rate[h.max_bin];
    h.min_rate = h.max_rate;
    int best   = h.max_bin;
    for (int i = 0; i < bins; ++i) {
        int idx = h.max_bin - i;
        if (idx < 0)
            idx += bins;
        if (h.rate[idx] < h.min_rate) {
            h.min_rate = h.rate[idx];
            best       = idx;
        }
    }
    h.offset_us = static_cast<int64_t>(best) * bin_us;
    return h;
}

} // namespace ebiv
