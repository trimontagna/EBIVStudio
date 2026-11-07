#pragma once

#include <filesystem>
#include <string>

#include "ebiv/processor.h"

namespace ebiv {

// NumPy .npz with one array "arr_0" of shape (nt, ny, nx, 6), float64, as written by ebiv_process.py /
// ebiv_run.py: [x, imgH - y, vx, vy, peak, event count] (y flipped, vx / vy raw, in px/ms).
// With `validated`, vx / vy are the post-processed values.
bool save_npz(const std::filesystem::path &path, const Result &result, bool validated, std::string *error);

// CSV, one line per vector: t0_us, x, y (sensor coordinates), vx, vy [px/ms], raw vx, vy, peak, events,
// valid, outlier, plus vx_ms / vy_ms [m/s] when a magnification is set.
bool save_csv(const std::filesystem::path &path, const Result &result, std::string *error);

// Raw .npy writer (little endian float64, C order).
std::string npy_bytes(const double *data, const std::vector<size_t> &shape);

} // namespace ebiv
