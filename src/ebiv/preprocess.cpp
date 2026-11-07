#include "ebiv/preprocess.h"

#include <algorithm>
#include <chrono>
#include <memory>

namespace ebiv {

bool any_stage(const PreprocessParams &p) {
    const evs::FilterConfig &f = p.filters;
    return p.hot_pixels || p.crop_roi || p.crop_time || f.activity_enabled || f.refractory_enabled ||
           f.trail_enabled || f.polarity_enabled;
}

bool preprocess(const EventSet &in, const PreprocessParams &p, EventSet &out, PreprocessReport &report,
                std::string *error, PreprocessProgress *progress) {
    const auto wall = std::chrono::steady_clock::now();
    report          = PreprocessReport();
    out             = EventSet();
    out.width       = in.width;
    out.height      = in.height;
    out.source      = in.source;
    out.format      = in.format;
    out.time_origin_us = in.time_origin_us;
    out.triggers       = in.triggers;

    size_t first = 0, last = in.events.size();
    if (p.crop_time) {
        const auto r = in.time_range(p.t_begin, p.t_end > p.t_begin ? p.t_end : int64_t(in.duration_us()) + 1);
        first = r.first;
        last  = r.second;
    }
    report.in = last - first;

    // pixel mask: hot pixels and the outside of the crop rectangle
    std::shared_ptr<std::vector<uint8_t>> mask;
    const size_t npix = size_t(in.width) * in.height;
    if (p.hot_pixels) {
        evs::PixelActivity activity(in.width, in.height);
        std::vector<evs::EventCD> chunk;
        for (size_t i = first; i < last; i += 1 << 20) {
            const size_t e = std::min(last, i + (1 << 20));
            chunk.clear();
            for (size_t k = i; k < e; ++k) {
                const Event &ev = in.events[k];
                chunk.emplace_back(ev.x, static_cast<unsigned short>(ev.y), static_cast<short>(ev.p),
                                   static_cast<evs::timestamp>(ev.t));
            }
            activity.add(chunk.data(), chunk.data() + chunk.size());
        }
        const auto hot = evs::detect_hot_pixels(activity, p.hot);
        report.hot_pixels = hot.size();
        mask = std::make_shared<std::vector<uint8_t>>(npix, 0);
        for (const auto &h : hot)
            (*mask)[size_t(h.y) * in.width + h.x] = 1;
    }
    if (p.crop_roi && p.roi.w > 0 && p.roi.h > 0) {
        if (!mask)
            mask = std::make_shared<std::vector<uint8_t>>(npix, 0);
        for (int y = 0; y < in.height; ++y)
            for (int x = 0; x < in.width; ++x)
                if (x < p.roi.x || x >= p.roi.x + p.roi.w || y < p.roi.y || y >= p.roi.y + p.roi.h)
                    (*mask)[size_t(y) * in.width + x] = 1;
    }

    evs::FilterPipeline pipeline(in.width, in.height);
    evs::FilterConfig config = p.filters;
    config.mask_enabled      = static_cast<bool>(mask);
    config.erc_enabled       = false;
    pipeline.set_config(config);
    if (mask)
        pipeline.set_mask(mask);

    out.events.reserve(last - first);
    const auto append = [&](const std::vector<evs::EventCD> &kept) {
        for (const auto &e : kept) {
            Event ev;
            ev.t = static_cast<uint32_t>(e.t);
            ev.x = e.x;
            ev.y = e.y;
            ev.p = e.p > 0 ? 1u : 0u;
            out.events.push_back(ev);
        }
    };
    std::vector<evs::EventCD> chunk;
    constexpr size_t kChunk = 1 << 20;
    for (size_t i = first; i < last; i += kChunk) {
        if (progress && progress->cancel) {
            if (error)
                *error = "Cancelled";
            out = EventSet();
            return false;
        }
        const size_t e = std::min(last, i + kChunk);
        chunk.clear();
        for (size_t k = i; k < e; ++k) {
            const Event &ev = in.events[k];
            chunk.emplace_back(ev.x, static_cast<unsigned short>(ev.y), static_cast<short>(ev.p),
                               static_cast<evs::timestamp>(ev.t));
        }
        append(pipeline.process(chunk.data(), chunk.data() + chunk.size()));
        if (progress)
            progress->fraction = static_cast<double>(e - first) / static_cast<double>(std::max<size_t>(1, last - first));
    }
    append(pipeline.flush());
    out.events.shrink_to_fit();
    report.out     = out.events.size();
    report.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall).count();
    if (progress)
        progress->fraction = 1.0;
    return true;
}

} // namespace ebiv
