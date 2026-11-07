#include "ebiv/processor.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include "ebiv/correlation.h"

namespace ebiv {

namespace {

// Per-thread scratch buffers and the evaluation of one sample for each method.
struct Evaluator {
    const Params &p;
    std::vector<double> vx, vy;
    std::vector<SampleEvent> ev_a, ev_b;
    std::vector<float> iwe, vol_a, vol_b, win_a, win_b, plane;
    ObjectiveMap map;
    ObjectivePeak peak;
    Correlator corr;
    int pairs = 0;

    explicit Evaluator(const Params &params) :
        p(params), vx(velocity_candidates(params.vx_min, params.vx_max, params.res_x)),
        vy(velocity_candidates(params.vy_min, params.vy_max, params.res_y)),
        corr(std::max(1, params.sample_w), std::max(1, params.sample_h)) {}

    Vector centre(int sx, int sy) const {
        Vector v;
        v.x = sx + p.sample_w * 0.5f;
        v.y = sy + p.sample_h * 0.5f;
        return v;
    }

    // ev_a holds the sample's events (time relative to the window start).
    Vector motion_comp(int sx, int sy) {
        Vector v = centre(sx, sy);
        peak     = ObjectivePeak();
        if (ev_a.empty() || vx.empty() || vy.empty())
            return v;
        objective_map(ev_a, static_cast<double>(p.t_sample), vx, vy, p.interpolation, p.objective, p.sample_w,
                      p.sample_h, map, iwe);
        peak = find_objective_peak(map, vx, vy, p.gauss_peak_fit);
        if (peak.ix < 0)
            return v;
        v.vx     = static_cast<float>(peak.vx);
        v.vy     = static_cast<float>(peak.vy);
        v.peak   = static_cast<float>(peak.value);
        v.events = static_cast<float>(peak.events);
        v.valid  = true;
        return v;
    }

    // ev_a / ev_b hold the events of the two sub-volumes.
    Vector corr_sum(int sx, int sy) {
        Vector v = centre(sx, sy);
        plane.clear();
        pairs = 0;
        if (ev_a.empty() || ev_b.empty())
            return v;
        const int w = p.sample_w, h = p.sample_h, nt = std::max(1, p.nt);
        const size_t slice = size_t(w) * h;
        const auto fill = [&](const std::vector<SampleEvent> &ev, std::vector<float> &vol) {
            vol.assign(slice * nt, 0.f);
            const double dt = static_cast<double>(std::max<int64_t>(1, p.t_sample));
            for (const SampleEvent &e : ev) {
                const int ti = std::min(nt - 1, static_cast<int>(std::floor(e.t * nt / dt)));
                vol[size_t(ti) * slice + size_t(e.y) * w + size_t(e.x)] = 1.f;
            }
        };
        fill(ev_a, vol_a);
        fill(ev_b, vol_b);
        corr.reset();
        for (int z = 0; z < nt; ++z)
            corr.add(vol_a.data() + z * slice, vol_b.data() + z * slice);
        pairs = corr.pairs();
        corr.result(nt, plane);
        if (pairs == 0)
            return v;
        const CorrPeak cp = find_corr_peak(plane, corr.plane_w(), corr.plane_h(), p.gauss_peak_fit);
        const double tsep = static_cast<double>(std::max<int64_t>(1, p.t_sep));
        v.vx     = static_cast<float>(cp.dx / tsep * 1000);
        v.vy     = static_cast<float>(cp.dy / tsep * 1000);
        v.peak   = static_cast<float>(cp.value);
        v.events = static_cast<float>((ev_a.size() + ev_b.size()) / 2);
        v.valid  = true;
        return v;
    }

    // win_a / win_b hold the interrogation windows; dt = time between the frames [us].
    Vector piv(int sx, int sy, double dt) {
        Vector v = centre(sx, sy);
        plane.clear();
        corr.reset();
        corr.add(win_a.data(), win_b.data());
        pairs = corr.pairs();
        corr.result(1, plane);
        if (pairs == 0)
            return v;
        const CorrPeak cp = find_corr_peak(plane, corr.plane_w(), corr.plane_h(), p.gauss_peak_fit);
        double sa = 0, sb = 0;
        for (size_t i = 0; i < win_a.size(); ++i) {
            sa += std::abs(win_a[i]);
            sb += std::abs(win_b[i]);
        }
        dt       = std::max(1.0, dt);
        v.vx     = static_cast<float>(cp.dx / dt * 1000);
        v.vy     = static_cast<float>(cp.dy / dt * 1000);
        v.peak   = static_cast<float>(cp.value);
        v.events = static_cast<float>((sa + sb) / 2);
        v.valid  = true;
        return v;
    }
};

void cut_window(const Image &img, int sx, int sy, int w, int h, std::vector<float> &out) {
    out.assign(size_t(w) * h, 0.f);
    for (int y = 0; y < h; ++y) {
        const int yy = sy + y;
        if (yy < 0 || yy >= img.h)
            continue;
        for (int x = 0; x < w; ++x) {
            const int xx = sx + x;
            if (xx < 0 || xx >= img.w)
                continue;
            const float v = img.at(xx, yy);
            out[size_t(y) * w + x] = std::isnan(v) ? 0.f : v;
        }
    }
}

void corr_sum_windows(const Params &p, int64_t t0, Interval &a, Interval &b) {
    a.t1 = t0 - p.t_sep / 2;
    a.t2 = a.t1 + p.t_sample;
    b.t1 = t0 + p.t_sep / 2;
    b.t2 = b.t1 + p.t_sample;
}

void collect(const EventSet &set, const Roi &roi, const Interval &iv, Polarity pol, std::vector<SampleEvent> &out) {
    out.clear();
    const auto [b, e] = set.time_range(iv.t1, iv.t2);
    for (size_t i = b; i < e; ++i) {
        const Event &ev = set.events[i];
        if (!matches(pol, ev) || ev.x < roi.x || ev.x >= roi.x + roi.w || ev.y < roi.y || ev.y >= roi.y + roi.h)
            continue;
        out.push_back({static_cast<float>(int64_t(ev.t) - iv.t1), static_cast<float>(ev.x - roi.x),
                       static_cast<float>(int(ev.y) - roi.y)});
    }
}

Image to_image(const std::vector<float> &data, int w, int h) {
    Image img;
    img.w    = w;
    img.h    = h;
    img.data = data;
    return img;
}

} // namespace

Result run_ebiv(const EventSet &set, const Params &params, ProcessProgress *progress) {
    const auto wall = std::chrono::steady_clock::now();
    Result result;
    result.params = params;
    result.width  = set.width;
    result.height = set.height;
    result.grid   = sampling_grid(params, set.width, set.height);
    result.source = set.source.u8string();
    const Grid &g = result.grid;
    const std::vector<int64_t> steps = time_steps(params);
    if (g.nx <= 0 || g.ny <= 0 || set.empty())
        return result;

    int threads = params.threads > 0 ? params.threads : static_cast<int>(std::thread::hardware_concurrency());
    threads     = std::clamp(threads, 1, 64);
    std::vector<std::unique_ptr<Evaluator>> evaluators;
    for (int i = 0; i < threads; ++i)
        evaluators.push_back(std::make_unique<Evaluator>(params));

    const uint64_t total = uint64_t(steps.size()) * g.nx * g.ny;
    std::atomic<uint64_t> done{0};

    for (size_t s = 0; s < steps.size(); ++s) {
        if (progress && progress->cancel)
            break;
        if (progress)
            progress->step = static_cast<int>(s);
        const int64_t t0 = steps[s];
        Field field;
        field.t0 = t0;
        field.v.resize(size_t(g.nx) * g.ny);

        std::unique_ptr<WindowIndex> index;
        Image img_a, img_b;
        Interval wa, wb;
        double piv_dt = 0;
        switch (params.method) {
        case Method::MotionComp:
            wa = {t0, t0 + params.t_sample};
            index = std::make_unique<WindowIndex>(set, wa.t1, wa.t2, params.polarity);
            break;
        case Method::CorrSum:
            corr_sum_windows(params, t0, wa, wb);
            index = std::make_unique<WindowIndex>(set, std::min(wa.t1, wb.t1), std::max(wa.t2, wb.t2),
                                                  params.polarity);
            break;
        default:
            piv_frames(params, t0, wa, wb);
            img_a  = render_pseudo_image(set, wa.t1, wa.t2, params.polarity, params.piv_image);
            img_b  = render_pseudo_image(set, wb.t1, wb.t2, params.polarity, params.piv_image);
            piv_dt = static_cast<double>(wb.t1 - wa.t1);
            break;
        }

        std::atomic<int> next_row{0};
        const auto worker = [&](Evaluator &ev) {
            while (true) {
                if (progress && progress->cancel)
                    return;
                const int r = next_row++;
                if (r >= g.ny)
                    return;
                const int sy = g.y0 + r * g.dy;
                for (int c = 0; c < g.nx; ++c) {
                    const int sx = g.x0 + c * g.dx;
                    Vector v;
                    switch (params.method) {
                    case Method::MotionComp:
                        ev.ev_a.clear();
                        index->query(sx, sy, params.sample_w, params.sample_h, wa.t1, wa.t2, ev.ev_a);
                        v = ev.motion_comp(sx, sy);
                        break;
                    case Method::CorrSum:
                        ev.ev_a.clear();
                        ev.ev_b.clear();
                        index->query(sx, sy, params.sample_w, params.sample_h, wa.t1, wa.t2, ev.ev_a);
                        index->query(sx, sy, params.sample_w, params.sample_h, wb.t1, wb.t2, ev.ev_b);
                        v = ev.corr_sum(sx, sy);
                        break;
                    default:
                        cut_window(img_a, sx, sy, params.sample_w, params.sample_h, ev.win_a);
                        cut_window(img_b, sx, sy, params.sample_w, params.sample_h, ev.win_b);
                        v = ev.piv(sx, sy, piv_dt);
                        break;
                    }
                    field.v[size_t(r) * g.nx + c] = v;
                }
                done += g.nx;
                if (progress)
                    progress->fraction = static_cast<double>(done) / static_cast<double>(total);
            }
        };
        std::vector<std::thread> pool;
        for (int i = 1; i < threads; ++i)
            pool.emplace_back(worker, std::ref(*evaluators[i]));
        worker(*evaluators[0]);
        for (auto &t : pool)
            t.join();
        if (progress && progress->cancel)
            break;
        result.fields.push_back(std::move(field));
    }
    result.samples = done;
    result.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall).count();
    if (progress)
        progress->fraction = 1.0;
    return result;
}

SampleDetail inspect_sample(const EventSet &set, const Params &params, int64_t t0, int sx, int sy) {
    SampleDetail d;
    d.method = params.method;
    d.roi    = {sx, sy, params.sample_w, params.sample_h};
    d.t0     = t0;
    Evaluator ev(params);
    const int w = params.sample_w, h = params.sample_h;

    switch (params.method) {
    case Method::MotionComp: {
        d.window_a = {t0, t0 + params.t_sample};
        collect(set, d.roi, d.window_a, params.polarity, ev.ev_a);
        d.result   = ev.motion_comp(sx, sy);
        d.events_a = static_cast<int>(ev.ev_a.size());
        d.vx       = ev.vx;
        d.vy       = ev.vy;
        d.map      = ev.map;
        d.peak     = ev.peak;
        std::vector<float> iwe;
        warp_events(ev.ev_a, static_cast<double>(params.t_sample), 0, 0, params.interpolation, w, h, iwe);
        d.iwe_zero = to_image(iwe, w, h);
        warp_events(ev.ev_a, static_cast<double>(params.t_sample), d.result.vx / 1000, d.result.vy / 1000,
                    params.interpolation, w, h, iwe);
        d.iwe_best = to_image(iwe, w, h);
        break;
    }
    case Method::CorrSum: {
        corr_sum_windows(params, t0, d.window_a, d.window_b);
        collect(set, d.roi, d.window_a, params.polarity, ev.ev_a);
        collect(set, d.roi, d.window_b, params.polarity, ev.ev_b);
        d.result   = ev.corr_sum(sx, sy);
        d.events_a = static_cast<int>(ev.ev_a.size());
        d.events_b = static_cast<int>(ev.ev_b.size());
        d.slices_used = ev.pairs;
        d.image_a  = Image(w, h);
        d.image_b  = Image(w, h);
        for (const auto &e : ev.ev_a)
            d.image_a.at(int(e.x), int(e.y)) += 1.f;
        for (const auto &e : ev.ev_b)
            d.image_b.at(int(e.x), int(e.y)) += 1.f;
        if (!ev.plane.empty())
            d.plane = to_image(ev.plane, 2 * w - 1, 2 * h - 1);
        break;
    }
    default: {
        piv_frames(params, t0, d.window_a, d.window_b);
        d.image_a = render_pseudo_image(set, d.window_a.t1, d.window_a.t2, params.polarity, params.piv_image, &d.roi);
        d.image_b = render_pseudo_image(set, d.window_b.t1, d.window_b.t2, params.polarity, params.piv_image, &d.roi);
        // windows at the sensor border are smaller than the sample: pad as the full-frame evaluation does
        cut_window(d.image_a, 0, 0, w, h, ev.win_a);
        cut_window(d.image_b, 0, 0, w, h, ev.win_b);
        d.image_a = to_image(ev.win_a, w, h);
        d.image_b = to_image(ev.win_b, w, h);
        d.result  = ev.piv(sx, sy, static_cast<double>(d.window_b.t1 - d.window_a.t1));
        for (float v : ev.win_a)
            d.events_a += v != 0.f;
        for (float v : ev.win_b)
            d.events_b += v != 0.f;
        if (!ev.plane.empty())
            d.plane = to_image(ev.plane, 2 * w - 1, 2 * h - 1);
        break;
    }
    }
    return d;
}

} // namespace ebiv
