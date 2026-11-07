// ebiv_cli: batch EBIV processing with the same library as EBIV Studio.
//   ebiv_cli recording.raw [--cfg FILE] [--method motion_comp|corr_sum|piv] [--t0 USEC]
//            [--interp nearest|bilinear|bilinear_python] [--threads N] [--out FILE.npz] [--csv FILE.csv] [--raw]

#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>

#include "ebiv/events.h"
#include "ebiv/export.h"
#include "ebiv/params.h"
#include "ebiv/postprocess.h"
#include "ebiv/processor.h"

using namespace ebiv;

int main(int argc, char **argv) {
    if (argc < 2 || !std::strcmp(argv[1], "--help") || !std::strcmp(argv[1], "-h")) {
        std::printf("Usage: ebiv_cli recording [--cfg FILE] [--method motion_comp|corr_sum|piv] [--t0 USEC]\n"
                    "                [--interp nearest|bilinear|bilinear_python] [--threads N]\n"
                    "                [--out FILE.npz] [--csv FILE.csv] [--raw]\n"
                    "  --raw  store the raw estimates in the .npz (default: after outlier replacement)\n");
        return 1;
    }
    const std::filesystem::path input = std::filesystem::u8path(argv[1]);
    Params p;
    std::string method, interp, error;
    std::filesystem::path npz, csv;
    long long t0 = -1;
    bool raw     = false;
    for (int i = 2; i < argc; ++i) {
        const std::string a = argv[i];
        const auto next     = [&]() -> std::string { return i + 1 < argc ? argv[++i] : ""; };
        if (a == "--cfg") {
            if (!load_cfg(std::filesystem::u8path(next()), p, &error)) {
                std::fprintf(stderr, "%s\n", error.c_str());
                return 1;
            }
        } else if (a == "--method")
            method = next();
        else if (a == "--t0")
            t0 = std::stoll(next());
        else if (a == "--interp")
            interp = next();
        else if (a == "--threads")
            p.threads = std::stoi(next());
        else if (a == "--out")
            npz = std::filesystem::u8path(next());
        else if (a == "--csv")
            csv = std::filesystem::u8path(next());
        else if (a == "--raw")
            raw = true;
        else {
            std::fprintf(stderr, "Unknown argument %s\n", a.c_str());
            return 1;
        }
    }
    if (!method.empty()) {
        bool found = false;
        for (int m = 0; m < static_cast<int>(Method::Count); ++m)
            if (method == to_string(static_cast<Method>(m))) {
                p.method = static_cast<Method>(m);
                found    = true;
            }
        if (!found) {
            std::fprintf(stderr, "Unknown method %s\n", method.c_str());
            return 1;
        }
    }
    if (!interp.empty() && !parse_interpolation(interp, p.interpolation)) {
        std::fprintf(stderr, "Unknown interpolation %s\n", interp.c_str());
        return 1;
    }
    if (t0 >= 0)
        p.t_start = t0;
    if (npz.empty())
        npz = input.parent_path() / (input.stem().u8string() + "_ebiv_" + to_string(p.method) + "_cpp.npz");

    auto wall = std::chrono::steady_clock::now();
    EventSet set;
    if (!load_events(input, set, &error)) {
        std::fprintf(stderr, "Loading failed: %s\n", error.c_str());
        return 1;
    }
    const double load_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall).count();
    std::printf("Loaded %s: %dx%d %s, %.2f M events, %.3f s (%.1f s to load)\n", input.u8string().c_str(), set.width,
                set.height, set.format.c_str(), set.events.size() / 1e6, set.duration_us() / 1e6, load_s);

    const Grid g = sampling_grid(p, set.width, set.height);
    std::printf("%s, t0 = %lld us, %d x %d samples of %d x %d px, %zu time step(s)\n", display_name(p.method),
                static_cast<long long>(p.t_start), g.nx, g.ny, p.sample_w, p.sample_h, time_steps(p).size());
    Result r = run_ebiv(set, p);
    PostParams post;
    for (Field &f : r.fields)
        validate(f, r.grid, post);
    std::printf("Processed %llu samples in %.2f s; %d outliers in the first step\n",
                static_cast<unsigned long long>(r.samples), r.seconds, r.fields.empty() ? 0 : r.fields[0].post.outliers);

    if (!save_npz(npz, r, !raw, &error)) {
        std::fprintf(stderr, "%s\n", error.c_str());
        return 1;
    }
    std::printf("Velocity data stored in: %s\n", npz.u8string().c_str());
    if (!csv.empty()) {
        if (!save_csv(csv, r, &error)) {
            std::fprintf(stderr, "%s\n", error.c_str());
            return 1;
        }
        std::printf("CSV stored in: %s\n", csv.u8string().c_str());
    }
    return 0;
}
