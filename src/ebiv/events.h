#pragma once

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

namespace ebiv {

// Compact event (8 bytes): whole recordings are kept in memory.
struct Event {
    uint32_t t;        // [us] relative to the first event of the recording
    uint16_t x;
    uint16_t y : 15;
    uint16_t p : 1;    // 1 = positive (ON), 0 = negative (OFF)
};
static_assert(sizeof(Event) == 8, "Event must stay 8 bytes");

struct Trigger {
    uint32_t t; // [us], same time base as the events
    int16_t p;
    int16_t id;
};

enum class Polarity { Positive, Negative, Both };
const char *to_string(Polarity polarity);   // "pos", "neg", "both" (ebiv_params.cfg spelling)
bool parse_polarity(const std::string &text, Polarity &polarity);

inline bool matches(Polarity polarity, const Event &e) {
    return polarity == Polarity::Both || (polarity == Polarity::Positive ? e.p != 0 : e.p == 0);
}

struct EventSet {
    int width = 0, height = 0;
    std::vector<Event> events;     // sorted by time
    std::vector<Trigger> triggers;
    int64_t time_origin_us = 0;    // time of t = 0 in the source file's time base
    std::filesystem::path source;
    std::string format;            // "EVT3", "HDF5", "EBIV evt", ...

    bool empty() const { return events.empty(); }
    uint32_t duration_us() const { return events.empty() ? 0 : events.back().t; }
    // Index range [first, last) of the events with t1 <= t < t2.
    std::pair<size_t, size_t> time_range(int64_t t1, int64_t t2) const;
    size_t memory_bytes() const { return events.capacity() * sizeof(Event) + triggers.capacity() * sizeof(Trigger); }
};

struct LoadProgress {
    std::atomic<double> fraction{0.0};
    std::atomic<uint64_t> events{0};
    std::atomic<bool> cancel{false};
};

// Loads a whole recording: .raw / .hdf5 / .dat through OpenEB, .evt (EBIV packed event format) natively.
// Times are shifted so that the first event is at t = 0.
bool load_events(const std::filesystem::path &path, EventSet &out, std::string *error,
                 LoadProgress *progress = nullptr);

// Writes the events with t1 <= t < t2 (t2 <= 0: until the end) in the EBIV .evt format read by
// ebiv_utils.EBILoadEvents() / pyebiv.
bool save_evt(const std::filesystem::path &path, const EventSet &set, int64_t t1, int64_t t2, std::string *error);

// Event rate over the recording [events / us], one value per `bin_us`.
std::vector<float> rate_timeline(const EventSet &set, int64_t bin_us);

} // namespace ebiv
