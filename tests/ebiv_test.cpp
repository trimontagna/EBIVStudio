#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <random>

#include "ebiv/correlation.h"
#include "ebiv/events.h"
#include "ebiv/export.h"
#include "ebiv/motion_comp.h"
#include "ebiv/params.h"
#include "ebiv/postprocess.h"
#include "ebiv/processor.h"
#include "ebiv/pulse.h"
#include "ebiv/sampling.h"

using namespace ebiv;

namespace {

std::filesystem::path temp_file(const std::string &name) {
    return std::filesystem::temp_directory_path() / ("ebiv_test_" + name);
}

// Particles (Gaussian-ish blobs of ~2 px) moving with a uniform velocity [px/ms]; every pixel a particle covers
// fires a positive event every `dt_us` while the particle is over it. Plus uniform noise.
EventSet moving_particles(double vx, double vy, int64_t duration_us, int count = 600, int width = 320, int height = 200,
                          double noise_rate = 0.0, unsigned seed = 1) {
    EventSet set;
    set.width  = width;
    set.height = height;
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> ux(0, width), uy(0, height), u01(0, 1);
    std::vector<std::pair<double, double>> p0(count);
    for (auto &p : p0)
        p = {ux(rng), uy(rng)};
    const int64_t dt_us = 50;
    for (int64_t t = 0; t < duration_us; t += dt_us) {
        for (const auto &p : p0) {
            const double x = p.first + vx * t / 1000.0, y = p.second + vy * t / 1000.0;
            const int px = static_cast<int>(std::floor(x)), py = static_cast<int>(std::floor(y));
            for (int dy = 0; dy <= 1; ++dy)
                for (int dx = 0; dx <= 1; ++dx) {
                    const int xx = px + dx, yy = py + dy;
                    if (xx < 0 || yy < 0 || xx >= width || yy >= height)
                        continue;
                    if (u01(rng) < 0.5) {
                        Event e;
                        e.t = static_cast<uint32_t>(t + static_cast<int64_t>(u01(rng) * dt_us));
                        e.x = static_cast<uint16_t>(xx);
                        e.y = static_cast<uint16_t>(yy);
                        e.p = 1;
                        set.events.push_back(e);
                    }
                }
        }
        const int noise = static_cast<int>(noise_rate * width * height * dt_us);
        for (int i = 0; i < noise; ++i) {
            Event e;
            e.t = static_cast<uint32_t>(t + static_cast<int64_t>(u01(rng) * dt_us));
            e.x = static_cast<uint16_t>(ux(rng));
            e.y = static_cast<uint16_t>(uy(rng));
            e.p = u01(rng) < 0.5 ? 1 : 0;
            set.events.push_back(e);
        }
    }
    std::stable_sort(set.events.begin(), set.events.end(), [](const Event &a, const Event &b) { return a.t < b.t; });
    return set;
}

// Median of the valid vectors of the first field.
void median_velocity(const Result &r, double &vx, double &vy, int &valid) {
    std::vector<double> x, y;
    for (const auto &v : r.fields.at(0).v)
        if (v.valid) {
            x.push_back(v.vx);
            y.push_back(v.vy);
        }
    valid = static_cast<int>(x.size());
    ASSERT_FALSE(x.empty());
    std::nth_element(x.begin(), x.begin() + x.size() / 2, x.end());
    std::nth_element(y.begin(), y.begin() + y.size() / 2, y.end());
    vx = x[x.size() / 2];
    vy = y[y.size() / 2];
}

} // namespace

TEST(EbivParams, VelocityCandidatesMatchNumpyArange) {
    // np.arange(-2, 5 + 0.25, 0.25) has 29 values
    const auto v = velocity_candidates(-2, 5, 0.25);
    ASSERT_EQ(v.size(), 29u);
    EXPECT_DOUBLE_EQ(v.front(), -2);
    EXPECT_DOUBLE_EQ(v.back(), 5);
    EXPECT_EQ(velocity_candidates(-3, 3, 0.25).size(), 25u);
    EXPECT_EQ(velocity_candidates(0, 1, 0.3).size(), 5u); // 0, .3, .6, .9, 1.2
}

TEST(EbivParams, GridMatchesPythonRange) {
    Params p;
    p.sample_w = p.sample_h = 40;
    p.step_x = p.step_y = 20;
    const Grid g = sampling_grid(p, 1280, 720);
    EXPECT_EQ(g.nx, 63); // range(0, 1280 - 40 + 1, 20)
    EXPECT_EQ(g.ny, 35); // range(0, 720 - 40 + 1, 20)
    p.roi_enabled = true;
    p.roi         = {100, 50, 200, 100};
    const Grid r  = sampling_grid(p, 1280, 720);
    EXPECT_EQ(r.x0, 100);
    EXPECT_EQ(r.nx, 9);
    EXPECT_EQ(r.ny, 4);
}

TEST(EbivParams, ReadsPythonConfigAndRoundTrips) {
    const auto path = temp_file("params.cfg");
    {
        std::ofstream out(path);
        out << "# EBIV processing parameters\n\n[DEFAULT]\ndoSingleTimeStep = False\nSampleTimeStart  = 12000\n"
               "SampleTimeEnd    = 50000\nSampleTimeStep   = 4000\nSampleDuration   = 8000\nSampleWidth      = 32\n"
               "SampleHeight     = 24\nSampleStepX      = 16\nSampleStepY      = 12\nEventPolarity    = both\n"
               "doGaussPeakFit   = False\n \n[MOTION_COMP]\nScanRangeVxMin   = -1.5 \nScanRangeVxMax   = 4\n"
               "ScanRangeVyMin   = -2\nScanRangeVyMax   = 2\nScanResolX       = 0.5\nScanResolY       = 0.25\n"
               "ObjectiveFxn    = SumExp\nInterpolation   = nearest\t\n\n[CORR_SUM]\nTimeOffset      = 2500\n"
               "NT              = 16\n\n[PIV]\nPIV_Delay       = 1500\n";
    }
    Params p;
    std::string error;
    ASSERT_TRUE(load_cfg(path, p, &error)) << error;
    EXPECT_FALSE(p.single_time_step);
    EXPECT_EQ(p.t_start, 12000);
    EXPECT_EQ(p.t_end, 50000);
    EXPECT_EQ(p.t_step, 4000);
    EXPECT_EQ(p.t_sample, 8000);
    EXPECT_EQ(p.sample_w, 32);
    EXPECT_EQ(p.sample_h, 24);
    EXPECT_EQ(p.polarity, Polarity::Both);
    EXPECT_FALSE(p.gauss_peak_fit);
    EXPECT_DOUBLE_EQ(p.vx_min, -1.5);
    EXPECT_EQ(p.objective, Objective::SumExp);
    EXPECT_EQ(p.interpolation, Interpolation::Nearest);
    EXPECT_EQ(p.t_sep, 2500);
    EXPECT_EQ(p.nt, 16);
    EXPECT_EQ(p.piv_delay, 1500);
    EXPECT_EQ(time_steps(p).size(), 10u);

    p.method        = Method::CorrSum;
    p.magnification = 12.5;
    p.roi_enabled   = true;
    p.roi           = {1, 2, 3, 4};
    ASSERT_TRUE(save_cfg(path, p, &error)) << error;
    Params q;
    ASSERT_TRUE(load_cfg(path, q, &error)) << error;
    EXPECT_EQ(q.method, Method::CorrSum);
    EXPECT_EQ(q.t_start, p.t_start);
    EXPECT_EQ(q.objective, p.objective);
    EXPECT_DOUBLE_EQ(q.magnification, 12.5);
    EXPECT_TRUE(q.roi_enabled);
    EXPECT_EQ(q.roi.h, 4);
    std::filesystem::remove(path);
}

TEST(EbivEvents, EvtRoundTrip) {
    EventSet set = moving_particles(1, 0, 2000, 50);
    set.time_origin_us = 123456;
    const auto path    = temp_file("events.evt");
    std::string error;
    ASSERT_TRUE(save_evt(path, set, 0, 0, &error)) << error;
    EXPECT_EQ(std::filesystem::file_size(path), 64 + set.events.size() * 8);
    EventSet in;
    ASSERT_TRUE(load_events(path, in, &error)) << error;
    ASSERT_EQ(in.events.size(), set.events.size());
    EXPECT_EQ(in.width, set.width);
    EXPECT_EQ(in.height, set.height);
    const uint32_t t0 = set.events.front().t;
    for (size_t i = 0; i < set.events.size(); i += 97) {
        EXPECT_EQ(in.events[i].t, set.events[i].t - t0);
        EXPECT_EQ(in.events[i].x, set.events[i].x);
        EXPECT_EQ(in.events[i].y, set.events[i].y);
        EXPECT_EQ(in.events[i].p, set.events[i].p);
    }
    std::filesystem::remove(path);
}

TEST(EbivSampling, WindowIndexMatchesBruteForce) {
    const EventSet set = moving_particles(2, 1, 5000, 400, 300, 180, 1e-4);
    const WindowIndex index(set, 1000, 4000, Polarity::Both, 16);
    std::mt19937 rng(3);
    for (int trial = 0; trial < 50; ++trial) {
        const int x = rng() % 280, y = rng() % 160, w = 5 + rng() % 40, h = 5 + rng() % 40;
        const int64_t t1 = 1000 + rng() % 1500, t2 = t1 + rng() % 1500;
        std::vector<SampleEvent> got;
        index.query(x, y, w, h, t1, t2, got);
        size_t expected = 0;
        for (const Event &e : set.events)
            if (e.t >= t1 && e.t < t2 && e.x >= x && e.x < x + w && e.y >= y && e.y < y + h)
                ++expected;
        EXPECT_EQ(got.size(), expected);
        for (const auto &s : got) {
            EXPECT_GE(s.x, 0);
            EXPECT_LT(s.x, w);
            EXPECT_GE(s.t, 0);
            EXPECT_LT(s.t, t2 - t1);
        }
    }
}

TEST(EbivCorrelation, MatchesDirectComputation) {
    const int w = 9, h = 7;
    std::mt19937 rng(5);
    std::uniform_real_distribution<float> u(0, 1);
    std::vector<float> a(w * h), b(w * h);
    for (auto &v : a)
        v = u(rng) < 0.3f ? 1.f : 0.f;
    for (auto &v : b)
        v = u(rng) < 0.3f ? 1.f : 0.f;
    Correlator corr(w, h);
    ASSERT_TRUE(corr.add(a.data(), b.data()));
    std::vector<float> plane;
    corr.result(1, plane);

    // fftconvolve(b - mb, flip(a - ma)) / (std a * std b) / (w * h)
    double ma = 0, mb = 0;
    for (int i = 0; i < w * h; ++i) {
        ma += a[i];
        mb += b[i];
    }
    ma /= w * h;
    mb /= w * h;
    double va = 0, vb = 0;
    for (int i = 0; i < w * h; ++i) {
        va += (a[i] - ma) * (a[i] - ma);
        vb += (b[i] - mb) * (b[i] - mb);
    }
    const double norm = std::sqrt(va / (w * h)) * std::sqrt(vb / (w * h)) * w * h;
    for (int sy = -(h - 1); sy <= h - 1; ++sy)
        for (int sx = -(w - 1); sx <= w - 1; ++sx) {
            double s = 0;
            for (int y = 0; y < h; ++y)
                for (int x = 0; x < w; ++x) {
                    const int xa = x - sx, ya = y - sy;
                    if (xa >= 0 && ya >= 0 && xa < w && ya < h)
                        s += (b[y * w + x] - mb) * (a[ya * w + xa] - ma);
                }
            EXPECT_NEAR(plane[(sy + h - 1) * (2 * w - 1) + sx + w - 1], s / norm, 1e-4) << sx << "," << sy;
        }
}

TEST(EbivCorrelation, PeakOfShiftedPattern) {
    const int w = 32, h = 32;
    std::mt19937 rng(7);
    std::vector<float> a(w * h, 0.f), b(w * h, 0.f);
    for (int i = 0; i < 60; ++i) {
        const int x = rng() % (w - 6), y = rng() % (h - 6);
        a[y * w + x]             = 1.f;
        b[(y + 2) * w + (x + 3)] = 1.f; // moved by (+3, +2)
    }
    Correlator corr(w, h);
    corr.add(a.data(), b.data());
    std::vector<float> plane;
    corr.result(1, plane);
    const CorrPeak p = find_corr_peak(plane, corr.plane_w(), corr.plane_h(), true);
    EXPECT_NEAR(p.dx, 3, 0.2);
    EXPECT_NEAR(p.dy, 2, 0.2);
}

TEST(EbivMotionComp, WarpConservesEventsAndObjectives) {
    std::vector<SampleEvent> ev = {{0, 5.25f, 5.75f}, {10, 2, 3}, {20, 7.5f, 1.5f}};
    std::vector<float> iwe;
    EXPECT_EQ(warp_events(ev, 20, 0, 0, Interpolation::Bilinear, 10, 10, iwe), 3);
    double sum = 0;
    for (float v : iwe)
        sum += v;
    EXPECT_NEAR(sum, 3.0, 1e-5);
    // correct bilinear: (5.25, 5.75) puts 0.75 * 0.25 on (5, 5)
    EXPECT_NEAR(iwe[5 * 10 + 5], 0.75f * 0.25f, 1e-6);
    warp_events(ev, 20, 0, 0, Interpolation::BilinearPython, 10, 10, iwe);
    EXPECT_NEAR(iwe[5 * 10 + 5], 0.25f * 0.75f, 1e-6); // Python: fx * fy
    EXPECT_NEAR(iwe[6 * 10 + 6], 0.75f * 0.25f, 1e-6); // Python: (1 - fx)(1 - fy)

    std::vector<float> img = {0, 1, 2, 3};
    EXPECT_NEAR(objective_value(img, Objective::Variance), 1.25, 1e-9);
    EXPECT_NEAR(objective_value(img, Objective::SumSq), 14, 1e-9);
    EXPECT_NEAR(objective_value(img, Objective::Isoa), 2, 1e-9);
    EXPECT_NEAR(objective_value(img, Objective::Moa), 3, 1e-9);
}

TEST(EbivMotionComp, RecoversUniformVelocity) {
    const EventSet set = moving_particles(1.6, -0.9, 12000);
    Params p;
    p.method   = Method::MotionComp;
    p.t_start  = 1000;
    p.t_sample = 10000;
    p.sample_w = p.sample_h = 40;
    p.step_x = p.step_y = 40;
    p.vx_min = -2;
    p.vx_max = 3;
    p.vy_min = -2;
    p.vy_max = 2;
    p.res_x = p.res_y = 0.25;
    const Result r = run_ebiv(set, p);
    ASSERT_EQ(r.fields.size(), 1u);
    double vx, vy;
    int valid;
    median_velocity(r, vx, vy, valid);
    EXPECT_GT(valid, static_cast<int>(r.fields[0].v.size()) / 2);
    EXPECT_NEAR(vx, 1.6, 0.12);
    EXPECT_NEAR(vy, -0.9, 0.12);
}

TEST(EbivCorrSum, RecoversUniformVelocity) {
    const EventSet set = moving_particles(2.0, 1.0, 20000);
    Params p;
    p.method   = Method::CorrSum;
    p.t_start  = 5000;
    p.t_sample = 6000;
    p.t_sep    = 3000; // 6 px, 3 px
    p.nt       = 20;
    p.sample_w = p.sample_h = 40;
    p.step_x = p.step_y = 40;
    const Result r = run_ebiv(set, p);
    double vx, vy;
    int valid;
    median_velocity(r, vx, vy, valid);
    EXPECT_NEAR(vx, 2.0, 0.1);
    EXPECT_NEAR(vy, 1.0, 0.1);
}

TEST(EbivPiv, RecoversUniformVelocity) {
    const EventSet set = moving_particles(-1.5, 0.5, 20000);
    Params p;
    p.method       = Method::Piv;
    p.t_start      = 8000;
    p.piv_delay    = 4000; // -6 px, 2 px
    p.piv_exposure = 300;
    p.piv_image    = PseudoMode::Binary;
    p.sample_w = p.sample_h = 40;
    p.step_x = p.step_y = 40;
    const Result r = run_ebiv(set, p);
    double vx, vy;
    int valid;
    median_velocity(r, vx, vy, valid);
    EXPECT_NEAR(vx, -1.5, 0.1);
    EXPECT_NEAR(vy, 0.5, 0.1);
    // the inspector evaluates a sample exactly as the grid run does
    const Vector &v0      = r.fields[0].v[5];
    const SampleDetail d  = inspect_sample(set, p, p.t_start, r.grid.x0 + 5 * r.grid.dx, r.grid.y0);
    EXPECT_FLOAT_EQ(d.result.vx, v0.vx);
    EXPECT_FLOAT_EQ(d.result.vy, v0.vy);
}

TEST(EbivPostprocess, MedianTestFlagsSpike) {
    const int nx = 8, ny = 6;
    Field f;
    Grid g;
    g.nx = nx;
    g.ny = ny;
    f.v.resize(nx * ny);
    for (int r = 0; r < ny; ++r)
        for (int c = 0; c < nx; ++c) {
            auto &v = f.v[r * nx + c];
            v.vx    = 1.0f + 0.01f * c;
            v.vy    = 0.5f;
            v.valid = true;
        }
    f.v[2 * nx + 3].vx = 8.f; // spike
    f.v[0].vy          = -6.f; // corner spike
    PostParams pp;
    validate(f, g, pp);
    EXPECT_EQ(f.post.outlier[2 * nx + 3], 1);
    EXPECT_EQ(f.post.outlier[0], 1);
    EXPECT_EQ(f.post.outliers, 2);
    EXPECT_NEAR(f.post.vx[2 * nx + 3], 1.03, 0.02);
    EXPECT_NEAR(f.post.vy[0], 0.5, 1e-5);
}

TEST(EbivPostprocess, VorticityOfSolidRotation) {
    // sensor coordinates (y down): u = w y, v = -w x is a counter-clockwise rotation seen with y up,
    // du/dy - dv/dx = 2 w
    Field f;
    Grid g;
    g.nx = g.ny = 7;
    g.dx = g.dy = 10;
    f.v.resize(49);
    f.post.vx.resize(49);
    f.post.vy.resize(49);
    const float w = 0.01f;
    for (int r = 0; r < 7; ++r)
        for (int c = 0; c < 7; ++c) {
            f.post.vx[r * 7 + c] = w * (r * 10.f);
            f.post.vy[r * 7 + c] = -w * (c * 10.f);
        }
    const auto vort = scalar_field(f, g, Scalar::Vorticity);
    EXPECT_NEAR(vort[3 * 7 + 3], 2 * w, 1e-6);
}

TEST(EbivPulse, FindsPulseOnset) {
    EventSet set;
    set.width = set.height = 64;
    const double period = 5000; // 200 Hz
    std::mt19937 rng(11);
    std::uniform_real_distribution<double> u(0, 1);
    for (int k = 0; k < 40; ++k)
        for (int i = 0; i < 2000; ++i) {
            // light on from 1200 us to 1700 us in each period, plus a floor
            const double t = k * period + (u(rng) < 0.85 ? 1200 + u(rng) * 500 : u(rng) * period);
            Event e;
            e.t = static_cast<uint32_t>(t);
            e.x = static_cast<uint16_t>(rng() % 64);
            e.y = static_cast<uint16_t>(rng() % 64);
            e.p = 1;
            set.events.push_back(e);
        }
    std::sort(set.events.begin(), set.events.end(), [](const Event &a, const Event &b) { return a.t < b.t; });
    const PulseHistogram h = pulse_histogram(set, 200, 50, 30, 2);
    ASSERT_EQ(h.rate.size(), 100u);
    EXPECT_GE(h.max_bin * 50, 1200);
    EXPECT_LT(h.max_bin * 50, 1700);
    EXPECT_TRUE(h.offset_us < 1200 || h.offset_us >= 1700); // minimum lies outside the light pulse
}

TEST(EbivExport, NpzHasNumpyLayout) {
    Result r;
    r.width  = 100;
    r.height = 80;
    r.grid.nx = 3;
    r.grid.ny = 2;
    Field f;
    f.v.resize(6);
    for (int i = 0; i < 6; ++i) {
        f.v[i].x  = 10.f * i;
        f.v[i].y  = 5.f;
        f.v[i].vx = 0.5f;
    }
    r.fields.push_back(f);
    const auto path = temp_file("result.npz");
    std::string error;
    ASSERT_TRUE(save_npz(path, r, false, &error)) << error;
    std::ifstream in(path, std::ios::binary);
    std::string bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    EXPECT_EQ(bytes.substr(0, 4), std::string("PK\x03\x04", 4));
    const auto npy = bytes.find("\x93NUMPY");
    ASSERT_NE(npy, std::string::npos);
    EXPECT_NE(bytes.find("'shape': (1, 2, 3, 6)"), std::string::npos);
    const uint16_t hlen = static_cast<uint8_t>(bytes[npy + 8]) | (static_cast<uint8_t>(bytes[npy + 9]) << 8);
    EXPECT_EQ((10 + hlen) % 64, 0);
    double first[6];
    std::memcpy(first, bytes.data() + npy + 10 + hlen, sizeof(first));
    EXPECT_DOUBLE_EQ(first[0], 0);
    EXPECT_DOUBLE_EQ(first[1], 75); // imgH - y
    EXPECT_DOUBLE_EQ(first[2], 0.5);
    in.close();
    std::filesystem::remove(path);
}
