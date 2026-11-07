#include "ebiv/postprocess.h"

#include <algorithm>
#include <cmath>

namespace ebiv {

namespace {

double median(std::vector<double> v) {
    const size_t n = v.size();
    std::sort(v.begin(), v.end());
    return n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

// np.gradient along one axis: central differences inside, one-sided at the borders.
float gradient(const std::vector<float> &f, int nx, int ny, int x, int y, bool along_x) {
    const int n = along_x ? nx : ny;
    const int i = along_x ? x : y;
    if (n < 2)
        return 0.f;
    const auto at = [&](int k) { return along_x ? f[size_t(y) * nx + k] : f[size_t(k) * nx + x]; };
    if (i == 0)
        return at(1) - at(0);
    if (i == n - 1)
        return at(n - 1) - at(n - 2);
    return 0.5f * (at(i + 1) - at(i - 1));
}

} // namespace

std::vector<uint8_t> normalized_median_test(const std::vector<float> &values, int nx, int ny, double threshold,
                                            double noise_floor) {
    std::vector<uint8_t> mask(values.size(), 0);
    if (nx < 3 || ny < 3)
        return mask;
    std::vector<double> roi;
    roi.reserve(8);
    for (int r = 0; r < ny; ++r) {
        for (int c = 0; c < nx; ++c) {
            // border vectors use the 3x3 block one row / column inward (as in the Python implementation)
            const int rc = std::clamp(r, 1, ny - 2), cc = std::clamp(c, 1, nx - 2);
            roi.clear();
            for (int dr = -1; dr <= 1; ++dr)
                for (int dc = -1; dc <= 1; ++dc)
                    if (rc + dr != r || cc + dc != c)
                        roi.push_back(values[size_t(rc + dr) * nx + cc + dc]);
            const double med = median(roi);
            std::vector<double> resid(roi.size());
            for (size_t i = 0; i < roi.size(); ++i)
                resid[i] = std::abs(roi[i] - med);
            const double resid_med = median(resid) + noise_floor;
            const double diff      = std::abs(values[size_t(r) * nx + c] - med);
            if (resid_med > 0 && diff / resid_med > threshold)
                mask[size_t(r) * nx + c] = 1;
        }
    }
    return mask;
}

void validate(Field &field, const Grid &grid, const PostParams &p) {
    const size_t n = field.v.size();
    Validated &out = field.post;
    out.vx.resize(n);
    out.vy.resize(n);
    out.outlier.assign(n, 0);
    out.outliers = out.replaced = 0;
    for (size_t i = 0; i < n; ++i) {
        out.vx[i] = field.v[i].vx;
        out.vy[i] = field.v[i].vy;
    }
    if (!p.enabled)
        return;

    for (size_t i = 0; i < n; ++i) {
        const Vector &v = field.v[i];
        if ((p.flag_empty && !v.valid) || (p.min_events > 0 && v.events < p.min_events) ||
            (p.min_peak > 0 && v.peak < p.min_peak) ||
            (p.max_magnitude > 0 && std::hypot(v.vx, v.vy) > p.max_magnitude))
            out.outlier[i] = 1;
    }
    if (p.median_threshold > 0) {
        std::vector<float> mag(n);
        for (size_t i = 0; i < n; ++i)
            mag[i] = std::hypot(out.vx[i], out.vy[i]);
        const auto mx = normalized_median_test(out.vx, grid.nx, grid.ny, p.median_threshold, p.noise_floor);
        const auto my = normalized_median_test(out.vy, grid.nx, grid.ny, p.median_threshold, p.noise_floor);
        const auto mm = normalized_median_test(mag, grid.nx, grid.ny, p.median_threshold, p.noise_floor);
        for (size_t i = 0; i < n; ++i)
            if (mx[i] + my[i] + mm[i] > 1)
                out.outlier[i] = 1;
    }
    for (uint8_t o : out.outlier)
        out.outliers += o;

    if (!p.replace || out.outliers == 0)
        return;
    // Replace outliers by the mean of their valid 8-neighbours, growing inward from the valid ones
    // (Python interpolates with scipy griddata).
    std::vector<uint8_t> known(n);
    for (size_t i = 0; i < n; ++i)
        known[i] = !out.outlier[i];
    bool progress = true;
    while (progress) {
        progress = false;
        std::vector<size_t> filled;
        for (int r = 0; r < grid.ny; ++r) {
            for (int c = 0; c < grid.nx; ++c) {
                const size_t i = size_t(r) * grid.nx + c;
                if (known[i])
                    continue;
                double sx = 0, sy = 0;
                int cnt = 0;
                for (int dr = -1; dr <= 1; ++dr) {
                    for (int dc = -1; dc <= 1; ++dc) {
                        const int rr = r + dr, cc = c + dc;
                        if ((dr == 0 && dc == 0) || rr < 0 || cc < 0 || rr >= grid.ny || cc >= grid.nx)
                            continue;
                        const size_t j = size_t(rr) * grid.nx + cc;
                        if (known[j]) {
                            sx += out.vx[j];
                            sy += out.vy[j];
                            ++cnt;
                        }
                    }
                }
                if (cnt > 0) {
                    out.vx[i] = static_cast<float>(sx / cnt);
                    out.vy[i] = static_cast<float>(sy / cnt);
                    filled.push_back(i);
                }
            }
        }
        for (size_t i : filled)
            known[i] = 1;
        out.replaced += static_cast<int>(filled.size());
        progress = !filled.empty();
    }
}

const char *to_string(Scalar scalar) {
    switch (scalar) {
    case Scalar::Magnitude:
        return "Velocity magnitude";
    case Scalar::Vx:
        return "Vx";
    case Scalar::Vy:
        return "Vy";
    case Scalar::Vorticity:
        return "Vorticity";
    case Scalar::Divergence:
        return "Divergence";
    case Scalar::Peak:
        return "Peak value";
    case Scalar::Events:
        return "Event count";
    default:
        return "?";
    }
}

std::vector<float> scalar_field(const Field &field, const Grid &g, Scalar scalar) {
    const size_t n = field.v.size();
    std::vector<float> out(n, 0.f);
    const auto &vx = field.post.vx.size() == n ? field.post.vx : std::vector<float>(n, 0.f);
    const auto &vy = field.post.vy.size() == n ? field.post.vy : std::vector<float>(n, 0.f);
    for (int r = 0; r < g.ny; ++r) {
        for (int c = 0; c < g.nx; ++c) {
            const size_t i = size_t(r) * g.nx + c;
            switch (scalar) {
            case Scalar::Magnitude:
                out[i] = std::hypot(vx[i], vy[i]);
                break;
            case Scalar::Vx:
                out[i] = vx[i];
                break;
            case Scalar::Vy:
                out[i] = vy[i];
                break;
            case Scalar::Vorticity:
                out[i] = gradient(vx, g.nx, g.ny, c, r, false) / g.dy - gradient(vy, g.nx, g.ny, c, r, true) / g.dx;
                break;
            case Scalar::Divergence:
                out[i] = gradient(vx, g.nx, g.ny, c, r, true) / g.dx + gradient(vy, g.nx, g.ny, c, r, false) / g.dy;
                break;
            case Scalar::Peak:
                out[i] = field.v[i].peak;
                break;
            case Scalar::Events:
                out[i] = field.v[i].events;
                break;
            default:
                break;
            }
        }
    }
    return out;
}

} // namespace ebiv
