#include "ebiv/correlation.h"

#include <algorithm>
#include <cmath>

#include "ebiv/motion_comp.h"

namespace ebiv {

Correlator::Correlator(int w, int h) : w_(w), h_(h) {
    nw_ = cv::getOptimalDFTSize(2 * w - 1);
    nh_ = cv::getOptimalDFTSize(2 * h - 1);
    pa_ = cv::Mat::zeros(nh_, nw_, CV_32F);
    pb_ = cv::Mat::zeros(nh_, nw_, CV_32F);
    reset();
}

void Correlator::reset() {
    acc_   = cv::Mat::zeros(nh_, nw_, CV_32F);
    pairs_ = 0;
}

bool Correlator::add(const float *a, const float *b) {
    const size_t n = size_t(w_) * h_;
    double sa = 0, sb = 0;
    for (size_t i = 0; i < n; ++i) {
        sa += a[i];
        sb += b[i];
    }
    const double ma = sa / n, mb = sb / n;
    if (!(ma * mb > 0))
        return false;
    double va = 0, vb = 0;
    for (size_t i = 0; i < n; ++i) {
        va += (a[i] - ma) * (a[i] - ma);
        vb += (b[i] - mb) * (b[i] - mb);
    }
    const double norm = std::sqrt(va / n) * std::sqrt(vb / n);
    if (!(norm > 0))
        return false;
    pa_.setTo(0);
    pb_.setTo(0);
    for (int y = 0; y < h_; ++y) {
        float *ra = pa_.ptr<float>(y);
        float *rb = pb_.ptr<float>(y);
        for (int x = 0; x < w_; ++x) {
            ra[x] = static_cast<float>(a[size_t(y) * w_ + x] - ma);
            rb[x] = static_cast<float>(b[size_t(y) * w_ + x] - mb);
        }
    }
    cv::dft(pa_, fa_, 0, h_);
    cv::dft(pb_, fb_, 0, h_);
    // B * conj(A): its inverse is sum_m a[m] b[m + s], i.e. fftconvolve(b, flip(a)) shifted to s = 0
    cv::mulSpectrums(fb_, fa_, prod_, 0, true);
    cv::scaleAdd(prod_, 1.0 / norm, acc_, acc_);
    ++pairs_;
    return true;
}

void Correlator::result(int slices, std::vector<float> &plane) {
    const int pw = plane_w(), ph = plane_h();
    plane.assign(size_t(pw) * ph, 0.f);
    if (pairs_ == 0)
        return;
    cv::dft(acc_, back_, cv::DFT_INVERSE | cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);
    const float scale = 1.f / (static_cast<float>(std::max(1, slices)) * w_ * h_);
    for (int ky = 0; ky < ph; ++ky) {
        const int sy  = ky - (h_ - 1);
        const float *row = back_.ptr<float>((sy + nh_) % nh_);
        for (int kx = 0; kx < pw; ++kx) {
            const int sx = kx - (w_ - 1);
            plane[size_t(ky) * pw + kx] = row[(sx + nw_) % nw_] * scale;
        }
    }
}

CorrPeak find_corr_peak(const std::vector<float> &plane, int pw, int ph, bool exp_fit) {
    CorrPeak peak;
    if (plane.empty())
        return peak;
    const size_t imax = static_cast<size_t>(std::max_element(plane.begin(), plane.end()) - plane.begin());
    const int ix = static_cast<int>(imax % pw), iy = static_cast<int>(imax / pw);
    peak.ix    = ix;
    peak.iy    = iy;
    peak.value = plane[imax];
    peak.dx    = ix - pw / 2;
    peak.dy    = iy - ph / 2;
    const auto at = [&](int i, int j) { return static_cast<double>(plane[size_t(j) * pw + i]); };
    if (ix > 0 && ix + 1 < pw)
        peak.dx = peak_fit_3pt(peak.dx, at(ix - 1, iy), at(ix, iy), at(ix + 1, iy), 1, exp_fit);
    if (iy > 0 && iy + 1 < ph)
        peak.dy = peak_fit_3pt(peak.dy, at(ix, iy - 1), at(ix, iy), at(ix, iy + 1), 1, exp_fit);
    return peak;
}

} // namespace ebiv
