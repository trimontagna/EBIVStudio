#include "ebiv/sampling.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ebiv {

Image render_pseudo_image(const EventSet &set, int64_t t1, int64_t t2, Polarity polarity, PseudoMode mode,
                          const Roi *roi) {
    int x0 = 0, y0 = 0, w = set.width, h = set.height;
    if (roi && roi->w > 0 && roi->h > 0) {
        x0 = std::clamp(roi->x, 0, set.width);
        y0 = std::clamp(roi->y, 0, set.height);
        w  = std::clamp(roi->x + roi->w, 0, set.width) - x0;
        h  = std::clamp(roi->y + roi->h, 0, set.height) - y0;
    }
    Image img(std::max(0, w), std::max(0, h),
              mode == PseudoMode::TimeSurface ? std::numeric_limits<float>::quiet_NaN() : 0.f);
    if (img.empty())
        return img;
    const auto [b, e] = set.time_range(t1, t2);
    for (size_t i = b; i < e; ++i) {
        const Event &ev = set.events[i];
        const int x = ev.x - x0, y = int(ev.y) - y0;
        if (x < 0 || y < 0 || x >= w || y >= h)
            continue;
        float &v = img.at(x, y);
        switch (mode) {
        case PseudoMode::Count:
            if (matches(polarity, ev))
                v += 1.f;
            break;
        case PseudoMode::Binary:
            if (matches(polarity, ev))
                v = 1.f;
            break;
        case PseudoMode::Signed:
            v += ev.p ? 1.f : -1.f;
            break;
        case PseudoMode::TimeSurface:
            if (matches(polarity, ev))
                v = static_cast<float>(int64_t(ev.t) - t1) * 1e-3f;
            break;
        default:
            break;
        }
    }
    return img;
}

void despeckle(Image &img, float empty) {
    if (img.w < 3 || img.h < 3)
        return;
    const bool nan_empty = std::isnan(empty);
    const auto is_set    = [&](float v) { return nan_empty ? !std::isnan(v) : v != empty; };
    const Image src      = img;
    for (int y = 0; y < img.h; ++y) {
        for (int x = 0; x < img.w; ++x) {
            if (!is_set(src.at(x, y)))
                continue;
            bool isolated = true;
            for (int dy = -1; dy <= 1 && isolated; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const int xx = x + dx, yy = y + dy;
                    if ((dx == 0 && dy == 0) || xx < 0 || yy < 0 || xx >= img.w || yy >= img.h)
                        continue;
                    if (is_set(src.at(xx, yy))) {
                        isolated = false;
                        break;
                    }
                }
            }
            if (isolated)
                img.at(x, y) = empty;
        }
    }
}

WindowIndex::WindowIndex(const EventSet &set, int64_t t1, int64_t t2, Polarity polarity, int cell) :
    width_(set.width), height_(set.height), cell_(std::max(1, cell)) {
    cols_ = (width_ + cell_ - 1) / cell_;
    rows_ = (height_ + cell_ - 1) / cell_;
    const size_t cells = size_t(cols_) * rows_;
    start_.assign(cells + 1, 0);
    const auto [b, e] = set.time_range(t1, t2);
    const auto cell_of = [&](const Event &ev) { return size_t(ev.y / cell_) * cols_ + ev.x / cell_; };
    // counting sort into cells: keeps the time order inside each cell
    for (size_t i = b; i < e; ++i) {
        const Event &ev = set.events[i];
        if (matches(polarity, ev) && ev.x < width_ && ev.y < height_)
            ++start_[cell_of(ev) + 1];
    }
    for (size_t c = 0; c < cells; ++c)
        start_[c + 1] += start_[c];
    items_.resize(start_[cells]);
    std::vector<uint32_t> fill(start_.begin(), start_.end() - 1);
    for (size_t i = b; i < e; ++i) {
        const Event &ev = set.events[i];
        if (matches(polarity, ev) && ev.x < width_ && ev.y < height_)
            items_[fill[cell_of(ev)]++] = {ev.t, ev.x, static_cast<uint16_t>(ev.y)};
    }
}

size_t WindowIndex::query(int x, int y, int w, int h, int64_t t1, int64_t t2, std::vector<SampleEvent> &out) const {
    const size_t before = out.size();
    const int xe = std::min(x + w, width_), ye = std::min(y + h, height_);
    const int xs = std::max(x, 0), ys = std::max(y, 0);
    if (xs >= xe || ys >= ye || t2 <= t1)
        return 0;
    const auto lower = [](const Item &it, int64_t t) { return static_cast<int64_t>(it.t) < t; };
    for (int cy = ys / cell_; cy <= (ye - 1) / cell_; ++cy) {
        for (int cx = xs / cell_; cx <= (xe - 1) / cell_; ++cx) {
            const size_t c   = size_t(cy) * cols_ + cx;
            const Item *beg  = items_.data() + start_[c];
            const Item *end  = items_.data() + start_[c + 1];
            const Item *it   = std::lower_bound(beg, end, t1, lower);
            const bool inner = cx * cell_ >= xs && (cx + 1) * cell_ <= xe && cy * cell_ >= ys && (cy + 1) * cell_ <= ye;
            for (; it != end && static_cast<int64_t>(it->t) < t2; ++it) {
                if (!inner && (it->x < xs || it->x >= xe || it->y < ys || it->y >= ye))
                    continue;
                out.push_back({static_cast<float>(static_cast<int64_t>(it->t) - t1), static_cast<float>(it->x - x),
                               static_cast<float>(it->y - y)});
            }
        }
    }
    return out.size() - before;
}

} // namespace ebiv
