#include "ebiv/motion_comp.h"

#include <algorithm>
#include <cmath>

namespace ebiv {

int warp_events(const std::vector<SampleEvent> &events, double dt, double vx, double vy, Interpolation interpolation,
                int w, int h, std::vector<float> &iwe) {
    iwe.assign(size_t(w) * h, 0.f);
    const double t_mean = dt / 2;
    int used            = 0;
    for (const SampleEvent &e : events) {
        const double wx = e.x - (e.t - t_mean) * vx;
        const double wy = e.y - (e.t - t_mean) * vy;
        if (interpolation == Interpolation::Nearest) {
            // np.round: round half to even, as std::nearbyint in the default rounding mode
            const int px = static_cast<int>(std::nearbyint(wx));
            const int py = static_cast<int>(std::nearbyint(wy));
            if (px >= 0 && py >= 0 && px < w && py < h) {
                iwe[size_t(py) * w + px] += 1.f;
                ++used;
            }
            continue;
        }
        const double fl = std::floor(wx), ft = std::floor(wy);
        const int px = static_cast<int>(fl), py = static_cast<int>(ft);
        if (px < 0 || py < 0 || px >= w - 1 || py >= h - 1)
            continue;
        const double fx = wx - fl, fy = wy - ft;
        float *row0 = &iwe[size_t(py) * w + px];
        float *row1 = row0 + w;
        if (interpolation == Interpolation::Bilinear) {
            row0[0] += static_cast<float>((1 - fx) * (1 - fy));
            row0[1] += static_cast<float>(fx * (1 - fy));
            row1[0] += static_cast<float>((1 - fx) * fy);
            row1[1] += static_cast<float>(fx * fy);
        } else { // weights of ebiv_utils.sumWarpedEvents()
            row0[0] += static_cast<float>(fy * fx);
            row0[1] += static_cast<float>(fy * (1 - fx));
            row1[0] += static_cast<float>((1 - fy) * fx);
            row1[1] += static_cast<float>((1 - fy) * (1 - fx));
        }
        ++used;
    }
    return used;
}

double objective_value(const std::vector<float> &iwe, Objective objective) {
    if (iwe.empty())
        return 0;
    const double n = static_cast<double>(iwe.size());
    switch (objective) {
    case Objective::Variance: {
        double s = 0, s2 = 0;
        for (float v : iwe) {
            s += v;
            s2 += double(v) * v;
        }
        const double mean = s / n;
        return std::max(0.0, s2 / n - mean * mean);
    }
    case Objective::SumSq: {
        double s2 = 0;
        for (float v : iwe)
            s2 += double(v) * v;
        return s2;
    }
    case Objective::SumExp: {
        double s = 0;
        for (float v : iwe)
            s += std::exp(double(v));
        return s;
    }
    case Objective::R1: {
        double s2 = 0, se = 0;
        for (float v : iwe) {
            s2 += double(v) * v;
            se += std::exp(-3.0 * v);
        }
        return (s2 / n) * se;
    }
    case Objective::Isoa: {
        double c = 0;
        for (float v : iwe)
            c += v > 1.f ? 1 : 0;
        return c;
    }
    case Objective::Moa:
        return *std::max_element(iwe.begin(), iwe.end());
    default:
        return 0;
    }
}

void objective_map(const std::vector<SampleEvent> &events, double dt, const std::vector<double> &vx,
                   const std::vector<double> &vy, Interpolation interpolation, Objective objective, int w, int h,
                   ObjectiveMap &map, std::vector<float> &iwe) {
    map.nx = static_cast<int>(vx.size());
    map.ny = static_cast<int>(vy.size());
    map.value.assign(size_t(map.nx) * map.ny, 0.0);
    map.events.assign(size_t(map.nx) * map.ny, 0);
    for (int j = 0; j < map.ny; ++j) {
        for (int i = 0; i < map.nx; ++i) {
            const int used = warp_events(events, dt, vx[i] / 1000, vy[j] / 1000, interpolation, w, h, iwe);
            map.value[size_t(j) * map.nx + i]  = objective_value(iwe, objective);
            map.events[size_t(j) * map.nx + i] = used;
        }
    }
}

double peak_fit_3pt(double x0, double a, double b, double c, double dx, bool exp_fit) {
    if (exp_fit && a > 0 && b > 0 && c > 0) {
        a = std::log(a);
        b = std::log(b);
        c = std::log(c);
    }
    const double denom = (a + c) * 2 - b * 4;
    if (denom == 0)
        return x0;
    return x0 + (a - c) / denom * dx;
}

ObjectivePeak find_objective_peak(const ObjectiveMap &map, const std::vector<double> &vx, const std::vector<double> &vy,
                                  bool exp_fit) {
    ObjectivePeak peak;
    if (map.value.empty())
        return peak;
    // flat map (e.g. no events): no estimate, as imgData2D.var() == 0 in Python
    const auto [mn, mx] = std::minmax_element(map.value.begin(), map.value.end());
    if (*mn == *mx)
        return peak;
    const size_t imax = static_cast<size_t>(mx - map.value.begin());
    const int ix = static_cast<int>(imax % map.nx), iy = static_cast<int>(imax / map.nx);
    peak.ix     = ix;
    peak.iy     = iy;
    peak.vx     = vx[ix];
    peak.vy     = vy[iy];
    peak.value  = *mx;
    peak.events = map.events[imax];
    const auto at = [&](int i, int j) { return map.value[size_t(j) * map.nx + i]; };
    if (ix > 0 && ix + 1 < map.nx)
        peak.vx = peak_fit_3pt(peak.vx, at(ix - 1, iy), at(ix, iy), at(ix + 1, iy), vx[1] - vx[0], exp_fit);
    if (iy > 0 && iy + 1 < map.ny)
        peak.vy = peak_fit_3pt(peak.vy, at(ix, iy - 1), at(ix, iy), at(ix, iy + 1), vy[1] - vy[0], exp_fit);
    return peak;
}

} // namespace ebiv
