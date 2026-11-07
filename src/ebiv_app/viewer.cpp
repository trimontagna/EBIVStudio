#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <fstream>

#include <imgui.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "app/platform.h"
#include "ebiv_app/app.h"

namespace ebiv::app {

using evs::app::to_utf8;

namespace {

constexpr ImU32 kRoiColor     = IM_COL32(255, 210, 60, 255);
constexpr ImU32 kGridColor    = IM_COL32(120, 200, 255, 160);
constexpr ImU32 kOutlierColor = IM_COL32(255, 50, 50, 255);
constexpr ImU32 kAxisColor    = IM_COL32(150, 150, 160, 255);
constexpr ImU32 kTextColor    = IM_COL32(220, 220, 225, 255);

bool diverging(Scalar s) {
    return s == Scalar::Vx || s == Scalar::Vy || s == Scalar::Vorticity || s == Scalar::Divergence;
}

void arrow(ImDrawList *dl, ImVec2 a, ImVec2 b, ImU32 color, float width) {
    const float dx = b.x - a.x, dy = b.y - a.y;
    const float len = std::sqrt(dx * dx + dy * dy);
    if (len < 0.5f) {
        dl->AddCircleFilled(a, width, color);
        return;
    }
    const float head = std::min(len * 0.35f, 4.f + 2.f * width);
    const float ux = dx / len, uy = dy / len;
    const ImVec2 base(b.x - ux * head, b.y - uy * head);
    dl->AddLine(a, base, color, width);
    dl->AddTriangleFilled(b, ImVec2(base.x - uy * head * 0.45f, base.y + ux * head * 0.45f),
                          ImVec2(base.x + uy * head * 0.45f, base.y - ux * head * 0.45f), color);
}

// Image -> texture with automatic contrast (symmetric for signed data).
void upload_image(Texture &tex, const Image &img, Colormap cm, bool symmetric, std::vector<uint8_t> &rgba) {
    if (img.empty())
        return;
    float lo, hi;
    robust_range(img.data, 0.5f, 99.5f, false, lo, hi);
    if (symmetric) {
        const float m = std::max(std::abs(lo), std::abs(hi));
        lo = -m;
        hi = m;
    }
    const Rgba nan = colormap(cm, 0.f);
    colorize(img.data.data(), img.w, img.h, lo, hi, cm, rgba, nan);
    tex.upload(rgba.data(), img.w, img.h, true);
}

// Axes frame for a 2D plot: maps (x, y) data coordinates into `rect` and draws ticks.
struct PlotFrame {
    ImVec2 pmin, pmax;
    double x0, x1, y0, y1;
    ImVec2 map(double x, double y) const {
        return ImVec2(static_cast<float>(pmin.x + (x - x0) / (x1 - x0) * (pmax.x - pmin.x)),
                      static_cast<float>(pmax.y - (y - y0) / (y1 - y0) * (pmax.y - pmin.y)));
    }
    void axes(ImDrawList *dl, const char *xlabel, const char *ylabel, double xdiv = 1) const {
        dl->AddRect(pmin, pmax, kAxisColor);
        const double sx = nice_step((x1 - x0) / xdiv, 8) * xdiv;
        for (double x = std::ceil(x0 / sx) * sx; x <= x1; x += sx) {
            const ImVec2 p = map(x, y0);
            dl->AddLine(p, ImVec2(p.x, p.y + 4), kAxisColor);
            dl->AddText(ImVec2(p.x - 10, p.y + 5), kTextColor, format_value(x / xdiv).c_str());
        }
        const double sy = nice_step(y1 - y0, 4);
        for (double y = std::ceil(y0 / sy) * sy; y <= y1; y += sy) {
            const ImVec2 p = map(x0, y);
            dl->AddLine(ImVec2(p.x - 4, p.y), p, kAxisColor);
            const std::string t = format_value(y);
            dl->AddText(ImVec2(p.x - 8 - ImGui::CalcTextSize(t.c_str()).x, p.y - 8), kTextColor, t.c_str());
        }
        if (xlabel)
            dl->AddText(ImVec2(pmax.x - ImGui::CalcTextSize(xlabel).x, pmax.y + 20), kTextColor, xlabel);
        if (ylabel)
            dl->AddText(ImVec2(pmin.x + 4, pmin.y + 2), kTextColor, ylabel);
    }
};

bool write_image(const std::filesystem::path &path, const cv::Mat &img, std::string *error) {
    std::string ext = path.extension().string();
    std::vector<uchar> buf;
    try {
        if (!cv::imencode(ext.empty() ? ".png" : ext, img, buf)) {
            if (error)
                *error = "Cannot encode " + ext;
            return false;
        }
    } catch (const cv::Exception &e) {
        if (error)
            *error = e.what();
        return false;
    }
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out.write(reinterpret_cast<const char *>(buf.data()), static_cast<std::streamsize>(buf.size()));
    out.close();
    if (!out) {
        if (error)
            *error = "Cannot write " + path.string();
        return false;
    }
    return true;
}

void pseudo_range(const Image &img, PseudoMode mode, int64_t duration_us, float &lo, float &hi) {
    switch (mode) {
    case PseudoMode::Binary:
        lo = 0;
        hi = 1;
        break;
    case PseudoMode::Signed: {
        robust_range(img.data, 0.5f, 99.5f, true, lo, hi);
        const float m = std::max(1.f, std::max(std::abs(lo), std::abs(hi)));
        lo = -m;
        hi = m;
        break;
    }
    case PseudoMode::TimeSurface:
        lo = 0;
        hi = std::max(1e-3f, duration_us * 1e-3f);
        break;
    default:
        robust_range(img.data, 0.f, 99.5f, true, lo, hi);
        lo = 0;
        hi = std::max(1.f, hi);
        break;
    }
}

} // namespace

// ------------------------------------------------------------------------------------------- helpers

void App::evaluation_windows(int64_t t0, Interval &a, Interval &b) const {
    switch (params_.method) {
    case Method::MotionComp:
        a = {t0, t0 + params_.t_sample};
        b = a;
        break;
    case Method::CorrSum:
        a.t1 = t0 - params_.t_sep / 2;
        a.t2 = a.t1 + params_.t_sample;
        b.t1 = t0 + params_.t_sep / 2;
        b.t2 = b.t1 + params_.t_sample;
        break;
    default:
        piv_frames(params_, t0, a, b);
        break;
    }
}

void App::current_window(int64_t &t1, int64_t &t2) const {
    if (!pseudo_.follow_evaluation) {
        t1 = pseudo_.t_us;
        t2 = pseudo_.t_us + pseudo_.duration_us;
        return;
    }
    const int64_t t0 = result_ && step_ < static_cast<int>(result_->fields.size()) ? result_->fields[step_].t0
                                                                                   : params_.t_start;
    Interval a, b;
    evaluation_windows(t0, a, b);
    const bool second = params_.method != Method::MotionComp && pseudo_.piv_frame == 1;
    t1 = second ? b.t1 : a.t1;
    t2 = second ? b.t2 : a.t2;
}

float App::unit_factor(const Result &r, float v) const {
    switch (display_.unit) {
    case VelocityUnit::DisplacementPx:
        return static_cast<float>(v * displacement_time_us(r.params) / 1000.0);
    case VelocityUnit::MetersPerSecond:
        return params_.magnification > 0 ? static_cast<float>(v / params_.magnification) : v;
    default:
        return v;
    }
}

const char *App::unit_label() const {
    switch (display_.unit) {
    case VelocityUnit::DisplacementPx:
        return "px";
    case VelocityUnit::MetersPerSecond:
        return params_.magnification > 0 ? "m/s" : "px/ms";
    default:
        return "px/ms";
    }
}

bool App::save_pseudo_image(const std::filesystem::path &path, const Image &img, bool colored) {
    if (img.empty())
        return false;
    std::string error;
    bool ok = false;
    if (colored && rgba_.size() == size_t(pseudo_tex_.width()) * pseudo_tex_.height() * 4) {
        cv::Mat rgba(pseudo_tex_.height(), pseudo_tex_.width(), CV_8UC4, rgba_.data()), bgr;
        cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);
        ok = write_image(path, bgr, &error);
    } else {
        cv::Mat f(img.h, img.w, CV_32F);
        for (int y = 0; y < img.h; ++y)
            for (int x = 0; x < img.w; ++x) {
                const float v   = img.at(x, y);
                f.at<float>(y, x) = std::isnan(v) ? 0.f : v;
            }
        ok = write_image(path, f, &error);
    }
    if (!ok)
        log("Saving the image failed: " + error, true);
    return ok;
}

void App::start_series_export(const std::filesystem::path &first) {
    if (!data_ || job_running_)
        return;
    auto data     = data_;
    auto s        = pseudo_;
    auto count    = series_count_;
    auto step     = series_step_us_;
    auto progress = std::make_shared<std::atomic<double>>(0.0);
    auto cancel   = std::make_shared<std::atomic<bool>>(false);
    auto written  = std::make_shared<int>(0);
    auto error    = std::make_shared<std::string>();
    std::string ext = first.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return std::tolower(c); });
    const bool png = ext == ".png";
    start_job(
        "Exporting pseudo images",
        [=] {
            std::vector<uint8_t> rgba;
            for (int i = 0; i < count && !*cancel; ++i) {
                const int64_t t1 = s.t_us + i * step;
                Image img = render_pseudo_image(*data, t1, t1 + s.duration_us, s.polarity, s.mode);
                if (s.despeckle)
                    despeckle(img, s.mode == PseudoMode::TimeSurface ? NAN : 0.f);
                char suffix[16];
                std::snprintf(suffix, sizeof(suffix), "_%04d", i);
                const auto path = first.parent_path() / (first.stem().u8string() + suffix + first.extension().u8string());
                cv::Mat out;
                if (png) {
                    float lo = s.lo, hi = s.hi;
                    if (s.auto_contrast)
                        pseudo_range(img, s.mode, s.duration_us, lo, hi);
                    colorize(img.data.data(), img.w, img.h, lo, hi, s.colormap, rgba, colormap(s.colormap, 0.f));
                    cv::Mat m(img.h, img.w, CV_8UC4, rgba.data());
                    cv::cvtColor(m, out, cv::COLOR_RGBA2BGR);
                } else {
                    out = cv::Mat(img.h, img.w, CV_32F);
                    for (int y = 0; y < img.h; ++y)
                        for (int x = 0; x < img.w; ++x)
                            out.at<float>(y, x) = std::isnan(img.at(x, y)) ? 0.f : img.at(x, y);
                }
                if (!write_image(path, out, error.get()))
                    return;
                ++*written;
                *progress = static_cast<double>(i + 1) / count;
            }
        },
        [=] {
            if (!error->empty())
                log("Series export failed: " + *error, true);
            log("Exported " + std::to_string(*written) + " pseudo images next to " + to_utf8(first));
        },
        [=] { return progress->load(); }, [=] { *cancel = true; });
}

void App::update_pseudo_image() {
    if (!data_)
        return;
    PseudoKey key;
    key.data = data_.get();
    current_window(key.t1, key.t2);
    key.overlay = pseudo_.follow_evaluation && params_.method != Method::MotionComp && pseudo_.piv_frame == 2;
    if (key.overlay) {
        const int64_t t0 = result_ && step_ < static_cast<int>(result_->fields.size()) ? result_->fields[step_].t0
                                                                                       : params_.t_start;
        Interval a, b;
        evaluation_windows(t0, a, b);
        key.t1 = a.t1;
        key.t2 = a.t2;
        key.t1b = b.t1;
        key.t2b = b.t2;
    }
    key.pol           = static_cast<int>(pseudo_.polarity);
    key.mode          = static_cast<int>(pseudo_.mode);
    key.cm            = static_cast<int>(pseudo_.colormap);
    key.despeckle     = pseudo_.despeckle;
    key.auto_contrast = pseudo_.auto_contrast;
    key.lo            = pseudo_.lo;
    key.hi            = pseudo_.hi;
    if (key == pseudo_key_ && pseudo_tex_.valid())
        return;
    pseudo_key_ = key;

    const float empty = pseudo_.mode == PseudoMode::TimeSurface ? NAN : 0.f;
    pseudo_img_ = render_pseudo_image(*data_, key.t1, key.t2, pseudo_.polarity, pseudo_.mode);
    if (pseudo_.despeckle)
        despeckle(pseudo_img_, empty);
    if (key.overlay) {
        // frame A red, frame B cyan, both white
        pseudo_img_b_ = render_pseudo_image(*data_, key.t1b, key.t2b, pseudo_.polarity, PseudoMode::Binary);
        Image a       = render_pseudo_image(*data_, key.t1, key.t2, pseudo_.polarity, PseudoMode::Binary);
        if (pseudo_.despeckle) {
            despeckle(a);
            despeckle(pseudo_img_b_);
        }
        rgba_.assign(size_t(a.w) * a.h * 4, 0);
        for (size_t i = 0; i < a.data.size(); ++i) {
            const bool ia = a.data[i] > 0, ib = pseudo_img_b_.data[i] > 0;
            uint8_t *p    = &rgba_[i * 4];
            p[0] = ia ? 255 : (ib ? 40 : 10);
            p[1] = ib ? 220 : (ia ? 50 : 10);
            p[2] = ib ? 255 : (ia ? 50 : 14);
            p[3] = 255;
        }
        pseudo_tex_.upload(rgba_.data(), a.w, a.h, true);
        return;
    }
    if (pseudo_.auto_contrast) {
        pseudo_range(pseudo_img_, pseudo_.mode, key.t2 - key.t1, pseudo_.lo, pseudo_.hi);
        pseudo_key_.lo = pseudo_.lo;
        pseudo_key_.hi = pseudo_.hi;
    }
    colorize(pseudo_img_.data.data(), pseudo_img_.w, pseudo_img_.h, pseudo_.lo, pseudo_.hi, pseudo_.colormap, rgba_,
             colormap(pseudo_.colormap, 0.f));
    pseudo_tex_.upload(rgba_.data(), pseudo_img_.w, pseudo_img_.h, true);
}

void App::update_result_background() {
    if (!data_ || !result_ || result_->fields.empty())
        return;
    BgKey key{data_.get(), result_.get(), step_, static_cast<int>(pseudo_.colormap)};
    if (key == bg_key_ && result_bg_tex_.valid())
        return;
    bg_key_ = key;
    const Params &p  = result_->params;
    const int64_t t0 = result_->fields[std::min<size_t>(step_, result_->fields.size() - 1)].t0;
    Interval a{t0, t0 + p.t_sample}, b;
    if (p.method == Method::CorrSum)
        a = {t0 - p.t_sep / 2, t0 - p.t_sep / 2 + p.t_sample};
    else if (p.method == Method::Piv)
        piv_frames(p, t0, a, b);
    const Image img = render_pseudo_image(*data_, a.t1, a.t2, p.polarity, PseudoMode::Count);
    float lo, hi;
    pseudo_range(img, PseudoMode::Count, a.t2 - a.t1, lo, hi);
    std::vector<uint8_t> rgba;
    const Colormap cm = pseudo_.colormap == Colormap::Gray || pseudo_.colormap == Colormap::GrayInverted
                            ? pseudo_.colormap
                            : Colormap::Gray;
    colorize(img.data.data(), img.w, img.h, lo, hi, cm, rgba, colormap(cm, 0.f));
    result_bg_tex_.upload(rgba.data(), img.w, img.h, true);
}

// Zoomable / pannable image area covering `size`. Shared zoom and pan between the tabs.
bool App::begin_image_view(const char *id, ImVec2 size, ViewTransform &view, bool &hovered) {
    const int w = data_ ? data_->width : 1280, h = data_ ? data_->height : 720;
    size.x = std::max(size.x, 50.f);
    size.y = std::max(size.y, 50.f);
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton(id, size,
                           ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight |
                               ImGuiButtonFlags_MouseButtonMiddle);
    ImGui::SetItemKeyOwner(ImGuiKey_MouseWheelY);
    hovered            = ImGui::IsItemHovered();
    const ImGuiIO &io  = ImGui::GetIO();
    const float fit    = std::min(size.x / w, size.y / h);
    const ImVec2 centre(p0.x + size.x * 0.5f, p0.y + size.y * 0.5f);
    const auto origin_of = [&](float zoom) {
        return ImVec2(centre.x - w * 0.5f * fit * zoom + pan_.x, centre.y - h * 0.5f * fit * zoom + pan_.y);
    };
    if (hovered && io.MouseWheel != 0) {
        const ImVec2 o0  = origin_of(zoom_);
        const float s0   = fit * zoom_;
        const ImVec2 img((io.MousePos.x - o0.x) / s0, (io.MousePos.y - o0.y) / s0);
        zoom_            = std::clamp(zoom_ * std::pow(1.2f, io.MouseWheel), 0.2f, 60.f);
        const float s1   = fit * zoom_;
        // keep the image point under the cursor in place
        pan_.x += io.MousePos.x - (centre.x - w * 0.5f * s1 + pan_.x + img.x * s1);
        pan_.y += io.MousePos.y - (centre.y - h * 0.5f * s1 + pan_.y + img.y * s1);
    }
    if (ImGui::IsItemActive() &&
        (ImGui::IsMouseDragging(ImGuiMouseButton_Right) || ImGui::IsMouseDragging(ImGuiMouseButton_Middle))) {
        pan_.x += io.MouseDelta.x;
        pan_.y += io.MouseDelta.y;
    }
    if (hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left) && !drawing_roi_) {
        zoom_ = 1.f;
        pan_  = ImVec2(0, 0);
    }
    view.scale  = fit * zoom_;
    view.origin = origin_of(zoom_);
    ImDrawList *dl = ImGui::GetWindowDrawList();
    dl->PushClipRect(p0, ImVec2(p0.x + size.x, p0.y + size.y), true);
    dl->AddRectFilled(p0, ImVec2(p0.x + size.x, p0.y + size.y), IM_COL32(18, 18, 22, 255));
    view_min_ = p0;
    view_max_ = ImVec2(p0.x + size.x, p0.y + size.y);
    return true;
}

void App::draw_sampling_overlay(ImDrawList *dl, const ViewTransform &view) {
    if (!data_)
        return;
    if (params_.roi_enabled)
        dl->AddRect(view.to_screen(float(params_.roi.x), float(params_.roi.y)),
                    view.to_screen(float(params_.roi.x + params_.roi.w), float(params_.roi.y + params_.roi.h)),
                    kRoiColor, 0, 0, 2.f);
    if (!show_grid_)
        return;
    const Grid g = sampling_grid(params_, data_->width, data_->height);
    if (g.nx <= 0 || g.ny <= 0)
        return;
    if (g.nx * g.ny <= 40000 && view.scale * std::min(g.dx, g.dy) >= 3) {
        for (int r = 0; r < g.ny; ++r)
            for (int c = 0; c < g.nx; ++c) {
                const ImVec2 p = view.to_screen(g.x0 + c * g.dx + params_.sample_w * 0.5f,
                                                g.y0 + r * g.dy + params_.sample_h * 0.5f);
                dl->AddRectFilled(ImVec2(p.x - 1, p.y - 1), ImVec2(p.x + 1, p.y + 1), kGridColor);
            }
    }
    // first sample window
    dl->AddRect(view.to_screen(float(g.x0), float(g.y0)),
                view.to_screen(float(g.x0 + params_.sample_w), float(g.y0 + params_.sample_h)), kGridColor, 0, 0, 1.5f);
}

// ------------------------------------------------------------------------------------------- viewer

void App::draw_viewer_window() {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(4, 4));
    const bool open = ImGui::Begin("Viewer", nullptr, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    ImGui::PopStyleVar();
    if (!open) {
        ImGui::End();
        return;
    }
    if (ImGui::BeginTabBar("viewer_tabs")) {
        const char *names[] = {"Pseudo image", "Results", "Pulse analysis"};
        for (int i = 0; i < 3; ++i) {
            const ImGuiTabItemFlags flags = select_tab_ == i ? ImGuiTabItemFlags_SetSelected : 0;
            if (ImGui::BeginTabItem(names[i], nullptr, flags)) {
                viewer_tab_ = i;
                if (i == 0)
                    tab_pseudo();
                else if (i == 1)
                    tab_results();
                else
                    tab_pulse();
                ImGui::EndTabItem();
            }
        }
        ImGui::EndTabBar();
    }
    select_tab_ = -1;
    if (capture_.pending) {
        capture_.min = view_min_;
        capture_.max = view_max_;
    }
    ImGui::End();
}

void App::draw_timeline(float height) {
    if (!data_ || timeline_.empty())
        return;
    const double dur = std::max(1.0, static_cast<double>(data_->duration_us()));
    if (!(tl_t1_ > tl_t0_)) {
        tl_t0_ = 0;
        tl_t1_ = dur;
    }
    const ImVec2 p0   = ImGui::GetCursorScreenPos();
    const ImVec2 size(ImGui::GetContentRegionAvail().x, height);
    ImGui::InvisibleButton("timeline", size, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
    ImGui::SetItemKeyOwner(ImGuiKey_MouseWheelY);
    const bool hovered = ImGui::IsItemHovered();
    const ImGuiIO &io  = ImGui::GetIO();
    ImDrawList *dl     = ImGui::GetWindowDrawList();
    const float left = 50, bottom = 18;
    const ImVec2 a(p0.x + left, p0.y + 4), b(p0.x + size.x - 8, p0.y + size.y - bottom);
    const float pw = std::max(1.f, b.x - a.x);
    const auto t_of = [&](float sx) { return tl_t0_ + (sx - a.x) / pw * (tl_t1_ - tl_t0_); };
    const auto x_of = [&](double t) { return static_cast<float>(a.x + (t - tl_t0_) / (tl_t1_ - tl_t0_) * pw); };

    if (hovered && io.MouseWheel != 0) {
        const double tm   = t_of(io.MousePos.x);
        const double span = std::clamp((tl_t1_ - tl_t0_) * std::pow(0.8, io.MouseWheel), 200.0, dur * 1.05);
        const double f    = (tm - tl_t0_) / (tl_t1_ - tl_t0_);
        tl_t0_ = tm - f * span;
        tl_t1_ = tl_t0_ + span;
    }
    if (ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
        const double dt = -io.MouseDelta.x / pw * (tl_t1_ - tl_t0_);
        tl_t0_ += dt;
        tl_t1_ += dt;
    }
    if (hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
        tl_t0_ = 0;
        tl_t1_ = dur;
    } else if (ImGui::IsItemActive() && ImGui::IsMouseDown(ImGuiMouseButton_Left) && io.MousePos.x >= a.x) {
        pseudo_.follow_evaluation = false;
        pseudo_.t_us = std::max<int64_t>(0, static_cast<int64_t>(t_of(io.MousePos.x)) - pseudo_.duration_us / 2);
    }

    dl->AddRectFilled(a, b, IM_COL32(24, 24, 30, 255));
    // evaluation range and windows
    if (!params_.single_time_step && params_.t_end > params_.t_start)
        dl->AddRectFilled(ImVec2(std::max(a.x, x_of(double(params_.t_start))), a.y),
                          ImVec2(std::min(b.x, x_of(double(params_.t_end))), b.y), IM_COL32(60, 90, 160, 60));
    {
        const int64_t t0 = result_ && step_ < static_cast<int>(result_->fields.size()) ? result_->fields[step_].t0
                                                                                       : params_.t_start;
        Interval wa, wb;
        evaluation_windows(t0, wa, wb);
        dl->AddRectFilled(ImVec2(std::max(a.x, x_of(double(wa.t1))), a.y),
                          ImVec2(std::min(b.x, std::max(x_of(double(wa.t1)) + 1, x_of(double(wa.t2)))), b.y),
                          IM_COL32(60, 200, 90, 70));
        if (params_.method != Method::MotionComp)
            dl->AddRectFilled(ImVec2(std::max(a.x, x_of(double(wb.t1))), a.y),
                              ImVec2(std::min(b.x, std::max(x_of(double(wb.t1)) + 1, x_of(double(wb.t2)))), b.y),
                              IM_COL32(60, 200, 220, 70));
    }

    // rate: max over the bins of each pixel column
    const double bin = static_cast<double>(timeline_bin_us_);
    std::vector<float> col(static_cast<size_t>(pw), 0.f);
    float ymax = 0;
    for (size_t i = 0; i < col.size(); ++i) {
        const double tb = t_of(a.x + i), te = t_of(a.x + i + 1);
        const long long kb = std::max(0ll, static_cast<long long>(tb / bin));
        const long long ke = std::min(static_cast<long long>(timeline_.size()) - 1, static_cast<long long>(te / bin));
        float m = 0;
        for (long long k = kb; k <= ke; ++k)
            m = std::max(m, timeline_[static_cast<size_t>(k)]);
        col[i] = m;
        ymax   = std::max(ymax, m);
    }
    ymax = std::max(ymax, 1e-6f);
    for (size_t i = 0; i < col.size(); ++i) {
        if (col[i] <= 0)
            continue;
        const float y = b.y - (b.y - a.y) * col[i] / ymax;
        dl->AddLine(ImVec2(a.x + i + 0.5f, b.y), ImVec2(a.x + i + 0.5f, y), IM_COL32(200, 200, 210, 220));
    }
    // triggers
    for (const auto &t : data_->triggers) {
        const float x = x_of(double(t.t));
        if (x >= a.x && x <= b.x)
            dl->AddLine(ImVec2(x, b.y - 6), ImVec2(x, b.y), t.p ? IM_COL32(255, 160, 40, 255) : IM_COL32(160, 100, 40, 255));
    }
    // pseudo-image window
    {
        int64_t t1, t2;
        current_window(t1, t2);
        const float x1 = x_of(double(t1)), x2 = std::max(x1 + 2, x_of(double(t2)));
        dl->AddRect(ImVec2(std::max(a.x - 1, x1), a.y), ImVec2(std::min(b.x + 1, x2), b.y), kRoiColor, 0, 0, 1.5f);
    }
    // axes
    dl->AddRect(a, b, kAxisColor);
    const double span = tl_t1_ - tl_t0_;
    const bool in_s   = span > 5e6;
    const double div  = in_s ? 1e6 : 1e3;
    const double step = nice_step(span / div, 10) * div;
    for (double t = std::ceil(tl_t0_ / step) * step; t <= tl_t1_; t += step) {
        const float x = x_of(t);
        dl->AddLine(ImVec2(x, b.y), ImVec2(x, b.y + 4), kAxisColor);
        dl->AddText(ImVec2(x - 12, b.y + 3), kTextColor, format_value(t / div).c_str());
    }
    dl->AddText(ImVec2(p0.x + 2, b.y + 3), kTextColor, in_s ? "t [s]" : "t [ms]");
    dl->AddText(ImVec2(p0.x + 2, a.y), kTextColor, format_value(ymax).c_str());
    dl->AddText(ImVec2(p0.x + 2, a.y + 16), IM_COL32(150, 150, 160, 255), "ev/us");
    if (hovered && io.MousePos.x >= a.x && io.MousePos.x <= b.x) {
        const double t = t_of(io.MousePos.x);
        const size_t k = static_cast<size_t>(std::clamp(t / bin, 0.0, double(timeline_.size() - 1)));
        dl->AddLine(ImVec2(io.MousePos.x, a.y), ImVec2(io.MousePos.x, b.y), IM_COL32(255, 255, 255, 90));
        ImGui::SetTooltip("t = %.3f ms\nrate = %.3f ev/us\n\nLeft: set pseudo-image start\nWheel: zoom, right drag: pan\n"
                          "Double click: whole recording",
                          t / 1000, timeline_[k]);
    }
}

void App::tab_pseudo() {
    if (!data_) {
        ImGui::TextDisabled("Open a recording (File > Open recording, or drop a file on the window).");
        return;
    }
    draw_timeline(110);
    update_pseudo_image();
    int64_t t1, t2;
    current_window(t1, t2);
    const auto [b, e] = data_->time_range(t1, t2);
    ImGui::Text("Window %.3f - %.3f ms  (%.3f ms)  |  %.0f k events", t1 / 1000.0, t2 / 1000.0, (t2 - t1) / 1000.0,
                (e - b) / 1000.0);
    if (drawing_roi_) {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1, 0.82f, 0.25f, 1), "  Drag on the image to draw the ROI");
        ImGui::SameLine();
        if (ImGui::SmallButton("Done"))
            drawing_roi_ = false;
    }

    ImVec2 avail = ImGui::GetContentRegionAvail();
    const bool overlay = pseudo_key_.overlay;
    const float bar_w  = overlay ? 0.f : 70.f;
    ViewTransform view;
    bool hovered = false;
    begin_image_view("pseudo_view", ImVec2(avail.x - bar_w, avail.y), view, hovered);
    ImDrawList *dl = ImGui::GetWindowDrawList();
    if (pseudo_tex_.valid())
        dl->AddImage(pseudo_tex_.id(), view.to_screen(0, 0), view.to_screen(float(data_->width), float(data_->height)));
    draw_sampling_overlay(dl, view);

    const ImGuiIO &io = ImGui::GetIO();
    if (drawing_roi_ && hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        roi_drag_  = true;
        roi_start_ = view.to_image(io.MousePos);
    }
    if (roi_drag_) {
        const ImVec2 cur = view.to_image(io.MousePos);
        const int x0 = std::clamp(static_cast<int>(std::floor(std::min(roi_start_.x, cur.x))), 0, data_->width - 1);
        const int y0 = std::clamp(static_cast<int>(std::floor(std::min(roi_start_.y, cur.y))), 0, data_->height - 1);
        const int x1 = std::clamp(static_cast<int>(std::ceil(std::max(roi_start_.x, cur.x))), x0 + 1, data_->width);
        const int y1 = std::clamp(static_cast<int>(std::ceil(std::max(roi_start_.y, cur.y))), y0 + 1, data_->height);
        params_.roi         = {x0, y0, x1 - x0, y1 - y0};
        params_.roi_enabled = true;
        if (!ImGui::IsMouseDown(ImGuiMouseButton_Left))
            roi_drag_ = false;
    }
    if (hovered) {
        const ImVec2 p = view.to_image(io.MousePos);
        const int x = static_cast<int>(std::floor(p.x)), y = static_cast<int>(std::floor(p.y));
        if (x >= 0 && y >= 0 && x < pseudo_img_.w && y < pseudo_img_.h) {
            char buf[96];
            std::snprintf(buf, sizeof(buf), "x %d  y %d  value %.4g", x, y, pseudo_img_.at(x, y));
            const ImVec2 pos(view_min_.x + 6, view_max_.y - 22);
            dl->AddRectFilled(ImVec2(pos.x - 3, pos.y - 2), ImVec2(pos.x + ImGui::CalcTextSize(buf).x + 3, pos.y + 18),
                              IM_COL32(0, 0, 0, 170));
            dl->AddText(pos, kTextColor, buf);
        }
    }
    dl->PopClipRect();
    if (!overlay) {
        ImGui::SameLine();
        const ImVec2 c = ImGui::GetCursorScreenPos();
        const char *label = pseudo_.mode == PseudoMode::TimeSurface ? "ms" : pseudo_.mode == PseudoMode::Signed ? "ON-OFF" : "events";
        draw_colorbar(dl, ImVec2(c.x + 8, c.y + 30), ImVec2(c.x + 24, c.y + std::min(avail.y - 10, 400.f)),
                      pseudo_.colormap, pseudo_.lo, pseudo_.hi, label);
        ImGui::Dummy(ImVec2(bar_w, 1));
    }
}

void App::tab_results() {
    if (!result_ || result_->fields.empty()) {
        ImGui::TextDisabled("No results yet: set the evaluation parameters and press Run EBIV (F5).");
        if (data_)
            ImGui::TextDisabled("The sampling grid and ROI are shown on the Pseudo image tab.");
        return;
    }
    const Result &r = *result_;
    step_ = std::clamp(step_, 0, static_cast<int>(r.fields.size()) - 1);
    if (r.fields.size() > 1) {
        if (ImGui::Button(playing_ ? "Pause" : "Play"))
            playing_ = !playing_;
        ImGui::SameLine();
        ImGui::SetNextItemWidth(260);
        if (ImGui::SliderInt("Time step", &step_, 0, static_cast<int>(r.fields.size()) - 1) && has_selection_)
            inspect(sel_col_, sel_row_);
        ImGui::SameLine();
    }
    const Field &f = r.fields[step_];
    const double t_ms = displacement_time_us(r.params) / 1000.0;
    ImGui::Text("%s  |  t0 = %.3f ms  |  %d x %d vectors  |  %d outliers  |  displacement over %.3g ms",
                display_name(r.params.method), f.t0 / 1000.0, r.grid.nx, r.grid.ny, f.post.outliers, t_ms);

    // scalar values in display units
    std::vector<float> vals = scalar_field(f, r.grid, display_.scalar);
    const bool velocity_scalar =
        display_.scalar == Scalar::Magnitude || display_.scalar == Scalar::Vx || display_.scalar == Scalar::Vy;
    const bool rate_scalar = display_.scalar == Scalar::Vorticity || display_.scalar == Scalar::Divergence;
    for (float &v : vals) {
        if (velocity_scalar)
            v = unit_factor(r, v);
        else if (rate_scalar)
            v *= 1000.f; // 1/ms -> 1/s
    }
    float lo = display_.lo, hi = display_.hi;
    if (display_.auto_range) {
        // range of the trusted vectors: outliers (even replaced ones) would stretch the colour scale
        std::vector<float> ok;
        ok.reserve(vals.size());
        for (size_t i = 0; i < vals.size(); ++i)
            if (f.v[i].valid && !(f.post.outlier.size() == vals.size() && f.post.outlier[i]))
                ok.push_back(vals[i]);
        if (ok.empty())
            ok = vals;
        robust_range(ok, 1.f, 99.f, false, lo, hi);
        // clusters of noisy vectors (sparse regions) pass the median test: clip to median +/- 5 robust sigma
        float med, mad;
        robust_range(ok, 50.f, 50.f, false, med, mad);
        std::vector<float> dev(ok.size());
        for (size_t i = 0; i < ok.size(); ++i)
            dev[i] = std::abs(ok[i] - med);
        float dmed, unused;
        robust_range(dev, 50.f, 50.f, false, dmed, unused);
        if (dmed > 0) {
            lo = std::max(lo, med - 5 * 1.4826f * dmed);
            hi = std::min(hi, med + 5 * 1.4826f * dmed);
        }
        if (diverging(display_.scalar)) {
            const float m = std::max(std::abs(lo), std::abs(hi));
            lo = -m;
            hi = m;
        } else if (display_.scalar != Scalar::Peak)
            lo = std::min(lo, 0.f);
        display_.lo = lo;
        display_.hi = hi;
    }
    const char *short_names[] = {"|V|", "Vx", "Vy", "Vort.", "Div.", "Peak", "Events"};
    const std::string label   = std::string(short_names[static_cast<int>(display_.scalar)]) +
                              (velocity_scalar ? std::string(" [") + unit_label() + "]" : rate_scalar ? " [1/s]" : "");

    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const float bar_w  = 90.f;
    ViewTransform view;
    bool hovered = false;
    begin_image_view("result_view", ImVec2(avail.x - bar_w, avail.y), view, hovered);
    ImDrawList *dl = ImGui::GetWindowDrawList();
    const Grid &g  = r.grid;
    const float hw = r.params.sample_w * 0.5f, hh = r.params.sample_h * 0.5f;

    if (display_.background == Background::PseudoImage) {
        update_result_background();
        if (result_bg_tex_.valid())
            dl->AddImage(result_bg_tex_.id(), view.to_screen(0, 0),
                         view.to_screen(float(data_ ? data_->width : r.width), float(data_ ? data_->height : r.height)));
    } else {
        dl->AddRect(view.to_screen(0, 0), view.to_screen(float(r.width), float(r.height)), IM_COL32(80, 80, 90, 255));
        if (display_.background == Background::ScalarMap) {
            const float inv = hi > lo ? 1.f / (hi - lo) : 1.f;
            for (int row = 0; row < g.ny; ++row)
                for (int c = 0; c < g.nx; ++c) {
                    const size_t i = size_t(row) * g.nx + c;
                    if (!f.v[i].valid && post_.flag_empty && !(post_.replace && post_.enabled))
                        continue;
                    const float cx = f.v[i].x, cy = f.v[i].y;
                    dl->AddRectFilled(view.to_screen(cx - g.dx * 0.5f, cy - g.dy * 0.5f),
                                      view.to_screen(cx + g.dx * 0.5f, cy + g.dy * 0.5f),
                                      colormap_u32(display_.colormap, (vals[i] - lo) * inv, display_.map_alpha));
                }
        }
    }

    // vectors
    if (display_.show_vectors) {
        std::vector<float> mags;
        mags.reserve(f.v.size());
        for (size_t i = 0; i < f.v.size(); ++i)
            if (f.v[i].valid && !(f.post.outlier.size() == f.v.size() && f.post.outlier[i]))
                mags.push_back(static_cast<float>(std::hypot(f.post.vx[i], f.post.vy[i]) * t_ms));
        if (display_.auto_scale && !mags.empty()) {
            // 95th percentile, but not beyond 3x the median (noisy clusters would shrink every arrow)
            float p95, med, unused;
            robust_range(mags, 95.f, 95.f, false, p95, unused);
            robust_range(mags, 50.f, 50.f, false, med, unused);
            const float ref = med > 0 ? std::min(p95, 3 * med) : p95;
            display_.vector_scale = ref > 1e-6f ? 1.2f * std::min(g.dx, g.dy) / ref : 1.f;
        }
        const float sc  = display_.vector_scale;
        const float inv = hi > lo ? 1.f / (hi - lo) : 1.f;
        for (size_t i = 0; i < f.v.size(); ++i) {
            const Vector &v   = f.v[i];
            const bool outlier = f.post.outlier.size() == f.v.size() && f.post.outlier[i];
            if (!v.valid && !display_.show_invalid && !(outlier && post_.replace && post_.enabled && f.post.replaced))
                continue;
            const float dx = static_cast<float>(f.post.vx[i] * t_ms) * sc;
            const float dy = static_cast<float>(f.post.vy[i] * t_ms) * sc;
            ImU32 color    = display_.vector_color == VectorColor::White   ? IM_COL32(255, 255, 255, 235)
                             : display_.vector_color == VectorColor::Black ? IM_COL32(0, 0, 0, 255)
                                                                           : colormap_u32(display_.colormap, (vals[i] - lo) * inv);
            if (outlier && display_.show_outliers)
                color = kOutlierColor;
            if (!v.valid && !outlier)
                color = IM_COL32(140, 140, 140, 200);
            arrow(dl, view.to_screen(v.x - dx * 0.5f, v.y - dy * 0.5f), view.to_screen(v.x + dx * 0.5f, v.y + dy * 0.5f),
                  color, display_.vector_width);
        }
    }
    draw_sampling_overlay(dl, view);
    if (has_selection_) {
        const float sx = float(g.x0 + sel_col_ * g.dx), sy = float(g.y0 + sel_row_ * g.dy);
        dl->AddRect(view.to_screen(sx, sy), view.to_screen(sx + 2 * hw, sy + 2 * hh), IM_COL32(255, 255, 0, 255), 0, 0, 2.f);
    }

    // hover / click
    const ImGuiIO &io = ImGui::GetIO();
    if (hovered && g.nx > 0 && g.ny > 0) {
        const ImVec2 p = view.to_image(io.MousePos);
        const int c    = std::clamp(static_cast<int>(std::lround((p.x - g.x0 - hw) / g.dx)), 0, g.nx - 1);
        const int row  = std::clamp(static_cast<int>(std::lround((p.y - g.y0 - hh) / g.dy)), 0, g.ny - 1);
        const size_t i = size_t(row) * g.nx + c;
        const Vector &v = f.v[i];
        const ImVec2 cs = view.to_screen(v.x, v.y);
        dl->AddCircle(cs, 5, IM_COL32(255, 255, 0, 200));
        if (ImGui::BeginTooltip()) {
            const float vx = f.post.vx[i], vy = f.post.vy[i];
            ImGui::Text("Sample [%d, %d] at x %.0f, y %.0f px", c, row, v.x, v.y);
            ImGui::Separator();
            ImGui::Text("vx %.4g  vy %.4g  |v| %.4g px/ms", vx, vy, std::hypot(vx, vy));
            ImGui::Text("displacement %.3g, %.3g px over %.3g ms", vx * t_ms, vy * t_ms, t_ms);
            if (params_.magnification > 0)
                ImGui::Text("velocity %.4g, %.4g m/s", vx / params_.magnification, vy / params_.magnification);
            ImGui::Text("peak %.4g  |  events %.0f", v.peak, v.events);
            if (!v.valid)
                ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1), "no estimate (no events / flat objective)");
            if (f.post.outlier[i])
                ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "outlier (raw vx %.4g, vy %.4g)", v.vx, v.vy);
            ImGui::TextDisabled("%s %.4g", label.c_str(), vals[i]);
            ImGui::TextDisabled("Click to inspect the sample");
            ImGui::EndTooltip();
        }
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
            inspect(c, row);
    }
    dl->PopClipRect();

    ImGui::SameLine();
    const ImVec2 cpos = ImGui::GetCursorScreenPos();
    if (display_.background == Background::ScalarMap || display_.vector_color == VectorColor::Scalar)
        draw_colorbar(dl, ImVec2(cpos.x + 8, cpos.y + 40), ImVec2(cpos.x + 24, cpos.y + std::min(avail.y - 10, 420.f)),
                      display_.colormap, lo, hi, label.c_str());
    ImGui::Dummy(ImVec2(bar_w, 1));
}

void App::tab_pulse() {
    if (pulse_.rate.empty() || !data_) {
        ImGui::TextDisabled("Pipeline > 4 Pulsed illumination > Analyse computes the mean event rate over one light "
                            "pulse period and the pulse onset.");
        return;
    }
    ImDrawList *dl     = ImGui::GetWindowDrawList();
    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const float ph     = (avail.y - 40) * 0.5f;

    // mean pulse shape
    ImGui::Text("Mean event rate over one period (%.2f Hz, %.1f us, %d periods, bin %d us)", pulse_.freq_hz,
                pulse_.period_us, pulse_.periods, pulse_.bin_us);
    ImVec2 p0 = ImGui::GetCursorScreenPos();
    ImGui::Dummy(ImVec2(avail.x, ph));
    PlotFrame fr{ImVec2(p0.x + 60, p0.y + 6), ImVec2(p0.x + avail.x - 10, p0.y + ph - 24), 0, pulse_.period_us, 0,
                 pulse_.max_rate * 1.08 + 1e-9};
    const float bw = std::max(1.f, (fr.pmax.x - fr.pmin.x) / pulse_.rate.size());
    for (size_t i = 0; i < pulse_.rate.size(); ++i) {
        const ImVec2 top = fr.map(static_cast<double>(i) * pulse_.bin_us, pulse_.rate[i]);
        dl->AddRectFilled(ImVec2(top.x, top.y), ImVec2(top.x + bw - (bw > 3 ? 1 : 0), fr.pmax.y), IM_COL32(210, 210, 220, 255));
    }
    const ImVec2 off = fr.map(double(pulse_.offset_us), 0);
    dl->AddLine(ImVec2(off.x, fr.pmin.y), ImVec2(off.x, fr.pmax.y), IM_COL32(255, 60, 60, 255), 2.f);
    dl->AddText(ImVec2(off.x + 4, fr.pmin.y + 2), IM_COL32(255, 90, 90, 255), "onset");
    fr.axes(dl, "time in period [us]", "ev/us");

    // event rate over a few periods with the pulse onsets
    const int periods = 6;
    const double t0   = std::max(0, pulse_start_) * pulse_.period_us;
    const double t1   = t0 + periods * pulse_.period_us;
    const int nb      = std::max(1, static_cast<int>((t1 - t0) / pulse_.bin_us));
    std::vector<float> rate(nb, 0.f);
    const auto [b, e] = data_->time_range(static_cast<int64_t>(t0), static_cast<int64_t>(t1));
    for (size_t i = b; i < e; ++i)
        if (matches(pulse_pol_, data_->events[i])) {
            const int k = static_cast<int>((data_->events[i].t - t0) / pulse_.bin_us);
            if (k >= 0 && k < nb)
                rate[k] += 1.f;
        }
    float rmax = 1e-9f;
    for (float &r : rate) {
        r /= static_cast<float>(pulse_.bin_us);
        rmax = std::max(rmax, r);
    }
    ImGui::Text("Event rate from %.3f ms with the estimated pulse onsets (red)", t0 / 1000);
    p0 = ImGui::GetCursorScreenPos();
    ImGui::Dummy(ImVec2(avail.x, ph));
    PlotFrame fr2{ImVec2(p0.x + 60, p0.y + 6), ImVec2(p0.x + avail.x - 10, p0.y + ph - 24), t0, t1, 0, rmax * 1.08};
    const float bw2 = std::max(1.f, (fr2.pmax.x - fr2.pmin.x) / nb);
    for (int i = 0; i < nb; ++i) {
        const ImVec2 top = fr2.map(t0 + i * pulse_.bin_us, rate[i]);
        dl->AddRectFilled(top, ImVec2(top.x + bw2 - (bw2 > 3 ? 1 : 0), fr2.pmax.y), IM_COL32(210, 210, 220, 255));
    }
    const double k0 = std::ceil((t0 - pulse_.offset_us) / pulse_.period_us);
    for (double t = pulse_.offset_us + k0 * pulse_.period_us; t < t1; t += pulse_.period_us) {
        const ImVec2 p = fr2.map(t, 0);
        dl->AddLine(ImVec2(p.x, fr2.pmin.y), ImVec2(p.x, fr2.pmax.y), IM_COL32(255, 60, 60, 220), 1.5f);
    }
    fr2.axes(dl, "t [ms]", "ev/us", 1000);
}

// ------------------------------------------------------------------------------------------- inspector

void App::inspect(int col, int row) {
    if (!result_ || !data_ || result_->fields.empty())
        return;
    const Grid &g = result_->grid;
    col           = std::clamp(col, 0, g.nx - 1);
    row           = std::clamp(row, 0, g.ny - 1);
    has_selection_ = true;
    sel_col_       = col;
    sel_row_       = row;
    const Field &f = result_->fields[std::clamp(step_, 0, static_cast<int>(result_->fields.size()) - 1)];
    detail_ = std::make_unique<SampleDetail>(
        inspect_sample(*data_, result_->params, f.t0, g.x0 + col * g.dx, g.y0 + row * g.dy));
    std::vector<uint8_t> rgba;
    const SampleDetail &d = *detail_;
    if (d.method == Method::MotionComp) {
        Image map;
        map.w    = d.map.nx;
        map.h    = d.map.ny;
        map.data.resize(d.map.value.size());
        std::transform(d.map.value.begin(), d.map.value.end(), map.data.begin(),
                       [](double v) { return static_cast<float>(v); });
        upload_image(insp_tex_[0], map, Colormap::Viridis, false, rgba);
        upload_image(insp_tex_[1], d.iwe_zero, Colormap::GrayInverted, false, rgba);
        upload_image(insp_tex_[2], d.iwe_best, Colormap::GrayInverted, false, rgba);
    } else {
        upload_image(insp_tex_[0], d.plane, Colormap::Jet, false, rgba);
        upload_image(insp_tex_[1], d.image_a, Colormap::GrayInverted, false, rgba);
        upload_image(insp_tex_[2], d.image_b, Colormap::GrayInverted, false, rgba);
    }
}

void App::draw_inspector_window() {
    if (!ImGui::Begin("Inspector")) {
        ImGui::End();
        return;
    }
    if (!has_selection_ || !detail_ || !result_) {
        ImGui::TextWrapped("Click a vector on the Results tab to see how its sample was evaluated: the objective map "
                           "and the warped event images (motion compensation), or the sub-volume projections and the "
                           "correlation plane (sum of correlation / PIV).");
        ImGui::End();
        return;
    }
    const SampleDetail &d = *detail_;
    const Vector &v       = d.result;
    const double t_ms     = displacement_time_us(result_->params) / 1000.0;
    ImGui::Text("Sample [%d, %d]  x %d..%d  y %d..%d", sel_col_, sel_row_, d.roi.x, d.roi.x + d.roi.w, d.roi.y,
                d.roi.y + d.roi.h);
    ImGui::Text("t0 = %.3f ms  |  %s", d.t0 / 1000.0, display_name(d.method));
    ImGui::Separator();
    if (v.valid) {
        ImGui::Text("vx %.4g  vy %.4g px/ms   |v| %.4g", v.vx, v.vy, std::hypot(v.vx, v.vy));
        ImGui::Text("displacement %.3g, %.3g px over %.3g ms", v.vx * t_ms, v.vy * t_ms, t_ms);
        if (params_.magnification > 0)
            ImGui::Text("velocity %.4g, %.4g m/s", v.vx / params_.magnification, v.vy / params_.magnification);
        ImGui::Text("peak %.4g  |  events %.0f", v.peak, v.events);
    } else
        ImGui::TextColored(ImVec4(1, 0.6f, 0.3f, 1), "No estimate for this sample (no events / flat objective)");
    if (ImGui::SmallButton("Re-evaluate"))
        inspect(sel_col_, sel_row_);
    ImGui::Separator();

    const float avail = ImGui::GetContentRegionAvail().x;
    ImDrawList *dl    = ImGui::GetWindowDrawList();
    const auto show = [&](const char *title, const Texture &tex, float width) -> std::pair<ImVec2, ImVec2> {
        ImGui::BeginGroup();
        ImGui::TextUnformatted(title);
        const float h   = tex.width() > 0 ? width * tex.height() / tex.width() : width;
        const ImVec2 p0 = ImGui::GetCursorScreenPos();
        if (tex.valid())
            ImGui::Image(tex.id(), ImVec2(width, h));
        else
            ImGui::Dummy(ImVec2(width, h));
        ImGui::EndGroup();
        return {p0, ImVec2(p0.x + width, p0.y + h)};
    };

    if (d.method == Method::MotionComp) {
        ImGui::Text("%d events, %d x %d velocity candidates", d.events_a, d.map.nx, d.map.ny);
        const auto [a, b] = show("Objective map (vx right, vy down)", insp_tex_[0], avail - 10);
        if (d.peak.ix >= 0 && d.vx.size() > 1 && d.vy.size() > 1) {
            // marker at the fitted peak, in candidate-cell coordinates
            const float fx = static_cast<float>((v.vx - d.vx.front()) / (d.vx[1] - d.vx[0]) + 0.5);
            const float fy = static_cast<float>((v.vy - d.vy.front()) / (d.vy[1] - d.vy[0]) + 0.5);
            const ImVec2 p(a.x + fx / d.map.nx * (b.x - a.x), a.y + fy / d.map.ny * (b.y - a.y));
            dl->AddLine(ImVec2(p.x - 6, p.y), ImVec2(p.x + 6, p.y), IM_COL32(255, 60, 60, 255), 2);
            dl->AddLine(ImVec2(p.x, p.y - 6), ImVec2(p.x, p.y + 6), IM_COL32(255, 60, 60, 255), 2);
            // zero velocity
            const float zx = static_cast<float>((0 - d.vx.front()) / (d.vx[1] - d.vx[0]) + 0.5);
            const float zy = static_cast<float>((0 - d.vy.front()) / (d.vy[1] - d.vy[0]) + 0.5);
            if (zx >= 0 && zx <= d.map.nx && zy >= 0 && zy <= d.map.ny)
                dl->AddCircle(ImVec2(a.x + zx / d.map.nx * (b.x - a.x), a.y + zy / d.map.ny * (b.y - a.y)), 4,
                              IM_COL32(255, 255, 255, 200));
        }
        ImGui::TextDisabled("vx %.3g .. %.3g, vy %.3g .. %.3g px/ms  (white: v = 0, red: peak)",
                            d.vx.empty() ? 0 : d.vx.front(), d.vx.empty() ? 0 : d.vx.back(),
                            d.vy.empty() ? 0 : d.vy.front(), d.vy.empty() ? 0 : d.vy.back());
        const float half = (avail - 18) * 0.5f;
        show("Events, no warp", insp_tex_[1], half);
        ImGui::SameLine();
        show("Warped at the estimate", insp_tex_[2], half);
        if (d.peak.ix >= 0) {
            const double o0 = objective_value(d.iwe_zero.data, result_->params.objective);
            const double o1 = objective_value(d.iwe_best.data, result_->params.objective);
            ImGui::TextDisabled("objective: %.4g -> %.4g (x %.2f)", o0, o1, o0 > 0 ? o1 / o0 : 0.0);
        }
    } else {
        if (d.method == Method::CorrSum)
            ImGui::Text("events A %d, B %d  |  %d of %d slice pairs correlated", d.events_a, d.events_b, d.slices_used,
                        result_->params.nt);
        else
            ImGui::Text("active pixels A %d, B %d", d.events_a, d.events_b);
        ImGui::TextDisabled("A [%.3f, %.3f) ms   B [%.3f, %.3f) ms", d.window_a.t1 / 1000.0, d.window_a.t2 / 1000.0,
                            d.window_b.t1 / 1000.0, d.window_b.t2 / 1000.0);
        const float half = (avail - 18) * 0.5f;
        show(d.method == Method::CorrSum ? "Sub-volume A (projection)" : "Window A", insp_tex_[1], half);
        ImGui::SameLine();
        show(d.method == Method::CorrSum ? "Sub-volume B (projection)" : "Window B", insp_tex_[2], half);
        if (!d.plane.empty()) {
            const auto [a, b] = show("Correlation plane (centre = no displacement)", insp_tex_[0], avail - 10);
            const float sx = (b.x - a.x) / d.plane.w, sy = (b.y - a.y) / d.plane.h;
            const ImVec2 c(a.x + (d.plane.w / 2 + 0.5f) * sx, a.y + (d.plane.h / 2 + 0.5f) * sy);
            dl->AddLine(ImVec2(c.x - 8, c.y), ImVec2(c.x + 8, c.y), IM_COL32(255, 255, 255, 160));
            dl->AddLine(ImVec2(c.x, c.y - 8), ImVec2(c.x, c.y + 8), IM_COL32(255, 255, 255, 160));
            if (v.valid) {
                const float ddx = static_cast<float>(v.vx * t_ms), ddy = static_cast<float>(v.vy * t_ms);
                const ImVec2 p(c.x + ddx * sx, c.y + ddy * sy);
                dl->AddCircle(p, 5, IM_COL32(255, 255, 255, 255), 0, 2);
            }
            ImGui::TextDisabled("peak at dx %.3g, dy %.3g px  (correlation %.4g)", v.vx * t_ms, v.vy * t_ms, v.peak);
        } else
            ImGui::TextDisabled("No correlation (empty or uniform windows)");
    }
    ImGui::End();
}

} // namespace ebiv::app
