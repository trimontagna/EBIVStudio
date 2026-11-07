#pragma once

#include <vector>

#include <opencv2/core.hpp>

namespace ebiv {

// Accumulates normalized cross-correlation planes of w x h slice pairs, as ebiv_utils.FFT_CrossCorr2D()
// followed by .sum(axis=0): each slice pair (with non-zero means) contributes
//     fftconvolve(b - mean(b), flip(a - mean(a))) / (std(a) * std(b))
// and the sum is divided by (slices * h * w). The plane is (2h - 1) x (2w - 1); its centre is zero displacement,
// a peak at +d means that the pattern of b is the one of a moved by +d.
// The spectra are summed and transformed back once, so adding a slice costs two forward DFTs.
class Correlator {
public:
    Correlator(int w, int h);

    void reset();
    // `a` / `b` hold w * h values. Returns false when the pair is skipped (a mean is 0).
    bool add(const float *a, const float *b);
    // Correlation plane (2h - 1) x (2w - 1), normalized by `slices` (FFT_CrossCorr2D divides by its nz).
    void result(int slices, std::vector<float> &plane);

    int plane_w() const { return 2 * w_ - 1; }
    int plane_h() const { return 2 * h_ - 1; }
    int pairs() const { return pairs_; }

private:
    int w_, h_, nw_, nh_;
    int pairs_ = 0;
    cv::Mat pa_, pb_, fa_, fb_, prod_, acc_, back_;
};

struct CorrPeak {
    double dx = 0, dy = 0; // [px]
    double value = 0;      // correlation at the integer peak
    int ix = -1, iy = -1;  // integer peak in the plane
};
// Highest value of the plane with 3-point sub-pixel fit (FindCorrPeak2D).
CorrPeak find_corr_peak(const std::vector<float> &plane, int pw, int ph, bool exp_fit);

} // namespace ebiv
