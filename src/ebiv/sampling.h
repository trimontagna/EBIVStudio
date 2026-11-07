#pragma once

#include <cstdint>
#include <vector>

#include "ebiv/events.h"
#include "ebiv/params.h"

namespace ebiv {

struct Image {
    int w = 0, h = 0;
    std::vector<float> data;

    Image() = default;
    Image(int width, int height, float value = 0.f) : w(width), h(height), data(size_t(width) * height, value) {}
    float &at(int x, int y) { return data[size_t(y) * w + x]; }
    float at(int x, int y) const { return data[size_t(y) * w + x]; }
    bool empty() const { return data.empty(); }
};

// Pseudo image of the events with t1 <= t < t2 of the given polarity, optionally restricted to `roi`
// (the image then covers the ROI only).
//  - Count: number of events per pixel           - Binary: 1 where at least one event
//  - Signed: ON - OFF count (polarity ignored)   - TimeSurface: time of the latest event [ms after t1],
//                                                  NaN where there is none
Image render_pseudo_image(const EventSet &set, int64_t t1, int64_t t2, Polarity polarity, PseudoMode mode,
                          const Roi *roi = nullptr);

// Removes isolated pixels (no set 8-neighbour), as EventImage::despeckle(). `empty` is the value of pixels
// without events (NaN for time surfaces).
void despeckle(Image &img, float empty = 0.f);

// Event of a sample, relative to the sample's top-left corner and to the start of its time window.
struct SampleEvent {
    float t, x, y;
};

// Events of one time window bucketed in square cells, so that the events of any rectangle and sub-interval
// can be fetched without scanning the whole window (the Python code runs np.where over all events per sample).
class WindowIndex {
public:
    WindowIndex(const EventSet &set, int64_t t1, int64_t t2, Polarity polarity, int cell = 16);

    // Events with x in [x, x + w), y in [y, y + h), t in [t1, t2), appended to `out` (relative coordinates,
    // time relative to t1). Returns the number appended.
    size_t query(int x, int y, int w, int h, int64_t t1, int64_t t2, std::vector<SampleEvent> &out) const;
    size_t size() const { return items_.size(); }

private:
    struct Item {
        uint32_t t;
        uint16_t x, y;
    };
    int width_, height_, cell_, cols_, rows_;
    std::vector<uint32_t> start_; // per cell, into items_ (cols_ * rows_ + 1 entries)
    std::vector<Item> items_;     // per cell, sorted by time
};

} // namespace ebiv
