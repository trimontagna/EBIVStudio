#include "ebiv/events.h"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <fstream>

#include <metavision/sdk/stream/event_file_reader.h>
#include <metavision/sdk/stream/raw_event_file_reader.h>

#include "io/event_file_source.h"

namespace ebiv {

const char *to_string(Polarity polarity) {
    switch (polarity) {
    case Polarity::Positive:
        return "pos";
    case Polarity::Negative:
        return "neg";
    default:
        return "both";
    }
}

bool parse_polarity(const std::string &text, Polarity &polarity) {
    std::string s = text;
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
    if (s == "pos" || s == "positive" || s == "+")
        polarity = Polarity::Positive;
    else if (s == "neg" || s == "negative" || s == "-")
        polarity = Polarity::Negative;
    else if (s == "both" || s == "all")
        polarity = Polarity::Both;
    else
        return false;
    return true;
}

std::pair<size_t, size_t> EventSet::time_range(int64_t t1, int64_t t2) const {
    const auto lower = [](const Event &e, int64_t t) { return static_cast<int64_t>(e.t) < t; };
    const auto b     = std::lower_bound(events.begin(), events.end(), t1, lower);
    const auto e     = std::lower_bound(b, events.end(), t2, lower);
    return {static_cast<size_t>(b - events.begin()), static_cast<size_t>(e - events.begin())};
}

namespace {

// EBIV packed event file (ebi_events.cpp: _EVENT_FILE_HDR, PACKED_EVENT).
#pragma pack(push, 1)
struct EvtHeader {
    uint32_t signature;   // "EVT3"
    uint32_t pad;         // alignment of the uint64 fields in the original struct
    uint64_t file_size;
    uint64_t event_count;
    uint64_t time_stamp;
    uint32_t duration;
    uint32_t header_length;
    uint32_t cols, rows;
    uint32_t reserved[4];
};
struct EvtPacked {
    uint16_t x, y;
    uint32_t time_pol; // time << 1 | polarity
};
#pragma pack(pop)
static_assert(sizeof(EvtHeader) == 64, "EVT header is 64 bytes");
constexpr uint32_t kEvtSignature = 0x33545645; // "EVT3"

bool load_evt(const std::filesystem::path &path, EventSet &out, std::string *error, LoadProgress *progress) {
    std::ifstream in(path, std::ios::binary);
    EvtHeader hdr{};
    if (!in || !in.read(reinterpret_cast<char *>(&hdr), 48)) {
        if (error)
            *error = "Cannot read " + path.string();
        return false;
    }
    if (hdr.signature != kEvtSignature) {
        if (error)
            *error = path.string() + " is not an EBIV event file (signature EVT3 expected)";
        return false;
    }
    in.seekg(hdr.header_length >= 48 ? hdr.header_length : 64);
    out.width  = static_cast<int>(hdr.cols);
    out.height = static_cast<int>(hdr.rows);
    out.format = "EBIV evt";
    out.events.reserve(hdr.event_count);

    std::vector<EvtPacked> buffer(1 << 20);
    uint64_t remaining = hdr.event_count;
    int64_t t_first    = -1;
    while (remaining > 0) {
        if (progress && progress->cancel) {
            if (error)
                *error = "Cancelled";
            return false;
        }
        const size_t n = static_cast<size_t>(std::min<uint64_t>(remaining, buffer.size()));
        in.read(reinterpret_cast<char *>(buffer.data()), static_cast<std::streamsize>(n * sizeof(EvtPacked)));
        const size_t got = static_cast<size_t>(in.gcount()) / sizeof(EvtPacked);
        for (size_t i = 0; i < got; ++i) {
            const EvtPacked &pe = buffer[i];
            const int64_t t     = pe.time_pol >> 1;
            if (t_first < 0)
                t_first = t;
            Event e;
            e.t = static_cast<uint32_t>(t - t_first);
            e.x = pe.x;
            e.y = pe.y;
            e.p = pe.time_pol & 1u;
            out.events.push_back(e);
        }
        remaining -= got;
        if (progress) {
            progress->events   = out.events.size();
            progress->fraction = 1.0 - static_cast<double>(remaining) / static_cast<double>(hdr.event_count);
        }
        if (got < n)
            break; // truncated file
    }
    out.time_origin_us = static_cast<int64_t>(hdr.time_stamp) + std::max<int64_t>(0, t_first);
    return true;
}

bool load_openeb(const std::filesystem::path &path, EventSet &out, std::string *error, LoadProgress *progress) {
    auto src = evs::EventFileSource::open(path, true, error);
    if (!src)
        return false;
    out.width  = src->width();
    out.height = src->height();
    out.format = src->encoding();
    // EVT 3.0 needs ~1.5-4 bytes per event: reserve generously to avoid copies of a multi-GB vector.
    if (src->raw_reader() && src->file_size() > 0)
        out.events.reserve(static_cast<size_t>(src->file_size() / 3));

    uint64_t bytes_read = 0;
    if (auto *raw = src->raw_reader())
        raw->add_raw_read_callback([&](const std::uint8_t *b, const std::uint8_t *e) { bytes_read += e - b; });
    evs::timestamp duration = -1;
    if (!src->raw_reader()) {
        try {
            duration = src->reader().get_duration();
        } catch (...) {}
    }

    int64_t t_first   = -1;
    bool sorted       = true;
    uint32_t last_t   = 0;
    bool time_overflow = false;
    src->reader().add_read_callback([&](const evs::EventCD *b, const evs::EventCD *e) {
        for (auto it = b; it != e; ++it) {
            if (t_first < 0)
                t_first = it->t;
            const int64_t t = it->t - t_first;
            if (t < 0 || t > 0xFFFFFFFFll) {
                time_overflow = true;
                continue;
            }
            Event ev;
            ev.t = static_cast<uint32_t>(t);
            ev.x = it->x;
            ev.y = it->y;
            ev.p = it->p > 0 ? 1u : 0u;
            if (ev.t < last_t)
                sorted = false;
            last_t = ev.t;
            out.events.push_back(ev);
        }
    });
    std::vector<evs::EventExtTrigger> triggers;
    src->reader().add_read_callback([&](const evs::EventExtTrigger *b, const evs::EventExtTrigger *e) {
        triggers.insert(triggers.end(), b, e);
    });

    try {
        while (true) {
            if (progress && progress->cancel) {
                if (error)
                    *error = "Cancelled";
                return false;
            }
            const bool more = src->reader().read();
            if (progress) {
                double f = 0;
                if (src->raw_reader() && src->file_size() > 0)
                    f = static_cast<double>(bytes_read) / static_cast<double>(src->file_size());
                else if (duration > 0 && !out.events.empty())
                    f = static_cast<double>(out.events.back().t) / static_cast<double>(duration);
                progress->fraction = std::clamp(f, 0.0, 1.0);
                progress->events   = out.events.size();
            }
            if (!more)
                break;
        }
    } catch (const std::exception &e) {
        if (error)
            *error = std::string("Error while reading ") + path.string() + ": " + e.what();
        return false;
    }

    if (!sorted)
        std::stable_sort(out.events.begin(), out.events.end(), [](const Event &a, const Event &b) { return a.t < b.t; });
    out.events.shrink_to_fit();
    for (const auto &tr : triggers) {
        const int64_t t = tr.t - std::max<int64_t>(0, t_first);
        if (t >= 0 && t <= 0xFFFFFFFFll)
            out.triggers.push_back({static_cast<uint32_t>(t), static_cast<int16_t>(tr.p), static_cast<int16_t>(tr.id)});
    }
    out.time_origin_us = std::max<int64_t>(0, t_first);
    if (time_overflow && error)
        *error = "Recording longer than 71 minutes: events beyond 2^32 us were dropped";
    return true;
}

} // namespace

bool load_events(const std::filesystem::path &path, EventSet &out, std::string *error, LoadProgress *progress) {
    out        = EventSet();
    out.source = path;
    std::string ext = path.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return std::tolower(c); });
    const bool ok = ext == ".evt" ? load_evt(path, out, error, progress) : load_openeb(path, out, error, progress);
    if (!ok)
        out = EventSet();
    else if (out.events.empty()) {
        if (error)
            *error = "No events in " + path.string();
        return false;
    }
    return ok;
}

bool save_evt(const std::filesystem::path &path, const EventSet &set, int64_t t1, int64_t t2, std::string *error) {
    if (t2 <= 0)
        t2 = static_cast<int64_t>(set.duration_us()) + 1;
    const auto [b, e] = set.time_range(t1, t2);
    if (b == e) {
        if (error)
            *error = "No events in the selected time range";
        return false;
    }
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) {
        if (error)
            *error = "Cannot create " + path.string();
        return false;
    }
    EvtHeader hdr{};
    hdr.signature     = kEvtSignature;
    hdr.event_count   = e - b;
    hdr.file_size     = sizeof(EvtHeader) + hdr.event_count * sizeof(EvtPacked);
    hdr.time_stamp    = static_cast<uint64_t>(set.time_origin_us);
    hdr.duration      = set.events[e - 1].t - set.events[b].t;
    hdr.header_length = sizeof(EvtHeader);
    hdr.cols          = static_cast<uint32_t>(set.width);
    hdr.rows          = static_cast<uint32_t>(set.height);
    out.write(reinterpret_cast<const char *>(&hdr), sizeof(hdr));

    std::vector<EvtPacked> buffer;
    buffer.reserve(1 << 20);
    for (size_t i = b; i < e; ++i) {
        const Event &ev = set.events[i];
        buffer.push_back({ev.x, static_cast<uint16_t>(ev.y), (ev.t << 1) | ev.p});
        if (buffer.size() == buffer.capacity() || i + 1 == e) {
            out.write(reinterpret_cast<const char *>(buffer.data()),
                      static_cast<std::streamsize>(buffer.size() * sizeof(EvtPacked)));
            buffer.clear();
        }
    }
    out.close();
    if (out.fail()) {
        if (error)
            *error = "Error while writing " + path.string();
        return false;
    }
    return true;
}

std::vector<float> rate_timeline(const EventSet &set, int64_t bin_us) {
    std::vector<float> rate;
    if (set.events.empty() || bin_us <= 0)
        return rate;
    rate.assign(static_cast<size_t>(set.duration_us() / bin_us) + 1, 0.f);
    for (const Event &e : set.events)
        rate[e.t / bin_us] += 1.f;
    const float scale = 1.f / static_cast<float>(bin_us);
    for (float &r : rate)
        r *= scale;
    return rate;
}

} // namespace ebiv
