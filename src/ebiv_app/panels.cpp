#include <algorithm>
#include <cmath>
#include <cstdio>

#include <imgui.h>

#include "app/platform.h"
#include "ebiv/export.h"
#include "ebiv_app/app.h"

namespace ebiv::app {

using evs::app::from_utf8;
using evs::app::open_file_dialog;
using evs::app::save_file_dialog;
using evs::app::to_utf8;

namespace {

constexpr float kField = 120.f; // width of numeric fields

void help(const char *text) {
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::BeginItemTooltip()) {
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 30.f);
        ImGui::TextUnformatted(text);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

// Time in [us] with its value in ms next to it.
bool input_us(const char *label, int64_t &v, int64_t vmin = 0, int64_t vmax = INT64_C(1) << 40) {
    ImGui::SetNextItemWidth(kField);
    const int64_t step = 100, fast = 1000;
    const bool changed = ImGui::InputScalar(label, ImGuiDataType_S64, &v, &step, &fast, "%lld");
    v = std::clamp(v, vmin, vmax);
    ImGui::SameLine();
    ImGui::TextDisabled("%.3g ms", v / 1000.0);
    return changed;
}

bool input_int(const char *label, int &v, int vmin, int vmax, int step = 1) {
    ImGui::SetNextItemWidth(kField);
    const bool changed = ImGui::InputInt(label, &v, step, step * 10);
    v = std::clamp(v, vmin, vmax);
    return changed;
}

bool input_double(const char *label, double &v, double step, const char *fmt = "%.3g") {
    ImGui::SetNextItemWidth(kField);
    return ImGui::InputDouble(label, &v, step, step * 10, fmt);
}

template <typename E> bool enum_combo(const char *label, E &value, int count, const char *(*name)(E)) {
    bool changed = false;
    if (ImGui::BeginCombo(label, name(value))) {
        for (int i = 0; i < count; ++i) {
            const E e = static_cast<E>(i);
            if (ImGui::Selectable(name(e), e == value)) {
                value   = e;
                changed = true;
            }
        }
        ImGui::EndCombo();
    }
    return changed;
}

const char *polarity_name(Polarity p) {
    switch (p) {
    case Polarity::Positive:
        return "Positive (ON)";
    case Polarity::Negative:
        return "Negative (OFF)";
    default:
        return "Both";
    }
}

bool polarity_combo(const char *label, Polarity &p) { return enum_combo(label, p, 3, polarity_name); }

bool roi_inputs(const char *id, Roi &roi, int width, int height) {
    ImGui::PushID(id);
    int v[4] = {roi.x, roi.y, roi.w, roi.h};
    ImGui::SetNextItemWidth(kField * 2);
    const bool changed = ImGui::InputInt4("x, y, w, h", v);
    if (changed) {
        roi.x = std::clamp(v[0], 0, std::max(0, width - 1));
        roi.y = std::clamp(v[1], 0, std::max(0, height - 1));
        roi.w = std::clamp(v[2], 1, std::max(1, width - roi.x));
        roi.h = std::clamp(v[3], 1, std::max(1, height - roi.y));
    }
    ImGui::PopID();
    return changed;
}

const char *trail_name(evs::TrailMode m) { return evs::to_string(m); }
const char *scalar_name(Scalar s) { return to_string(s); }
const char *colormap_name(Colormap c) { return to_string(c); }
const char *method_name(Method m) { return display_name(m); }
const char *objective_name(Objective o) {
    switch (o) {
    case Objective::Variance:
        return "var - variance";
    case Objective::SumSq:
        return "SumSq - sum of squares";
    case Objective::SumExp:
        return "SumExp - sum of exponentials";
    case Objective::R1:
        return "R1 (Stoffregen et al.)";
    case Objective::Isoa:
        return "isoa - inverse sum of accumulations";
    case Objective::Moa:
        return "MOA - max of accumulations";
    default:
        return "?";
    }
}
const char *interpolation_name(Interpolation i) {
    switch (i) {
    case Interpolation::Nearest:
        return "Nearest";
    case Interpolation::Bilinear:
        return "Bilinear";
    case Interpolation::BilinearPython:
        return "Bilinear (Python-compatible)";
    default:
        return "?";
    }
}
const char *pseudo_name(PseudoMode m) { return to_string(m); }
const char *background_name(Background b) {
    switch (b) {
    case Background::None:
        return "None";
    case Background::PseudoImage:
        return "Pseudo image";
    case Background::ScalarMap:
        return "Scalar map";
    default:
        return "?";
    }
}
const char *unit_name(VelocityUnit u) {
    switch (u) {
    case VelocityUnit::PxPerMs:
        return "Velocity [px/ms]";
    case VelocityUnit::DisplacementPx:
        return "Displacement [px]";
    case VelocityUnit::MetersPerSecond:
        return "Velocity [m/s]";
    default:
        return "?";
    }
}
const char *vector_color_name(VectorColor c) {
    switch (c) {
    case VectorColor::Scalar:
        return "Colormap";
    case VectorColor::White:
        return "White";
    case VectorColor::Black:
        return "Black";
    default:
        return "?";
    }
}

bool header(const char *label, bool open_default = true) {
    return ImGui::CollapsingHeader(label, open_default ? ImGuiTreeNodeFlags_DefaultOpen : 0);
}

} // namespace

void App::draw_pipeline_window() {
    if (!ImGui::Begin("Pipeline")) {
        ImGui::End();
        return;
    }
    draw_job_progress();
    section_recording();
    section_preprocess();
    section_pseudo();
    section_pulse();
    section_evaluation();
    section_postprocess();
    section_display();
    section_export();
    ImGui::End();
}

void App::draw_job_progress() {
    if (!job_running_)
        return;
    const double f = job_fraction_ ? job_fraction_() : 0;
    const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - job_start_).count();
    ImGui::TextUnformatted(job_label_.c_str());
    char overlay[64];
    std::snprintf(overlay, sizeof(overlay), "%.0f %%  (%.0f s)", f * 100, s);
    ImGui::ProgressBar(static_cast<float>(f), ImVec2(-80, 0), overlay);
    ImGui::SameLine();
    if (ImGui::Button("Cancel") && job_cancel_)
        job_cancel_();
    ImGui::Separator();
}

// ------------------------------------------------------------------------------------------- 1 recording

void App::section_recording() {
    if (!header("1  Recording"))
        return;
    ImGui::BeginDisabled(job_running_);
    if (ImGui::Button("Open recording...")) {
        const auto path =
            open_file_dialog("Open recording", "Event recordings|*.raw;*.hdf5;*.h5;*.dat;*.evt|All files|*.*");
        if (!path.empty())
            open_file(path);
    }
    ImGui::EndDisabled();
    if (!raw_) {
        ImGui::TextDisabled("No recording loaded (.raw / .hdf5 / .dat via OpenEB, EBIV .evt).");
        return;
    }
    ImGui::SameLine();
    if (ImGui::Button("Save events (.evt)...")) {
        const auto path = save_file_dialog("Save events (EBIV .evt format)", "EBIV events|*.evt",
                                           data_->source.parent_path() / (data_->source.stem().u8string() + ".evt"),
                                           ".evt");
        std::string error;
        if (!path.empty()) {
            if (save_evt(path, *data_, 0, 0, &error))
                log("Events saved to " + to_utf8(path) + (preprocessed_ ? " (pre-processed)" : ""));
            else
                log(error, true);
        }
    }
    help("Writes the current (pre-processed) events in the packed format read by ebiv_utils.EBILoadEvents().");

    if (ImGui::BeginTable("info", 2, ImGuiTableFlags_SizingFixedFit)) {
        const auto row = [](const char *k, const std::string &v) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextDisabled("%s", k);
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(v.c_str());
        };
        char buf[128];
        row("File", to_utf8(raw_->source.filename()));
        std::snprintf(buf, sizeof(buf), "%d x %d  (%s)", raw_->width, raw_->height, raw_->format.c_str());
        row("Sensor", buf);
        std::snprintf(buf, sizeof(buf), "%.3f s", raw_->duration_us() / 1e6);
        row("Duration", buf);
        std::snprintf(buf, sizeof(buf), "%.2f M  (%.2f Mev/s)", raw_->events.size() / 1e6,
                      raw_->duration_us() ? raw_->events.size() / (raw_->duration_us() * 1.0) : 0.0);
        row("Events", buf);
        if (preprocessed_) {
            std::snprintf(buf, sizeof(buf), "%.2f M after pre-processing", data_->events.size() / 1e6);
            row("", buf);
        }
        row("Triggers", std::to_string(raw_->triggers.size()));
        row("Memory", evs::app::human_bytes(static_cast<double>(raw_->memory_bytes() +
                                                                 (preprocessed_ ? data_->memory_bytes() : 0))));
        ImGui::EndTable();
    }
}

// ------------------------------------------------------------------------------------------- 2 pre-processing

void App::section_preprocess() {
    if (!header("2  Pre-processing", false))
        return;
    const int w = raw_ ? raw_->width : 1280, h = raw_ ? raw_->height : 720;
    evs::FilterConfig &f = pre_.filters;
    ImGui::PushID("pre");

    ImGui::Checkbox("Remove hot pixels", &pre_.hot_pixels);
    help("Detects pixels much more active than their neighbourhood over the loaded recording "
         "(EventStudio hot pixel detector) and removes their events.");
    if (pre_.hot_pixels) {
        ImGui::Indent();
        input_double("Min. rate [Hz]", pre_.hot.min_rate_hz, 5);
        input_double("Ratio to neighbours", pre_.hot.local_ratio, 1);
        ImGui::Unindent();
    }

    ImGui::Checkbox("Activity (noise) filter", &f.activity_enabled);
    help("Keeps an event only if enough pixels of its neighbourhood fired within +/- the window "
         "(removes background noise, keeps particle images).");
    if (f.activity_enabled) {
        ImGui::Indent();
        input_us("Window [us]##act", f.activity_window_us, 1, 1000000);
        input_int("Radius [px]", f.activity_radius, 1, 5);
        input_int("Min. neighbours", f.activity_min_neighbors, 1, 24);
        ImGui::Checkbox("Same polarity", &f.activity_same_polarity);
        ImGui::SameLine();
        bool la = f.activity_look_ahead;
        if (ImGui::Checkbox("Look-ahead", &la))
            f.activity_look_ahead = la;
        ImGui::Unindent();
    }

    ImGui::Checkbox("Refractory period", &f.refractory_enabled);
    if (f.refractory_enabled) {
        ImGui::Indent();
        input_us("Period [us]##refr", f.refractory_us, 1, 10000000);
        ImGui::Unindent();
    }

    ImGui::Checkbox("Trail filter", &f.trail_enabled);
    help("TRAIL keeps the first event of each burst on a pixel, STC_CUT_TRAIL the second only, "
         "STC_KEEP_TRAIL the second and following ones.");
    if (f.trail_enabled) {
        ImGui::Indent();
        ImGui::SetNextItemWidth(kField * 1.6f);
        enum_combo("Mode##trail", f.trail_mode, 3, trail_name);
        input_us("Threshold [us]##trail", f.trail_threshold_us, 1, 100000000);
        ImGui::Unindent();
    }

    ImGui::Checkbox("Polarity filter", &f.polarity_enabled);
    if (f.polarity_enabled) {
        ImGui::Indent();
        ImGui::Checkbox("Keep ON", &f.keep_on);
        ImGui::SameLine();
        ImGui::Checkbox("Keep OFF", &f.keep_off);
        ImGui::Unindent();
    }

    ImGui::Checkbox("Crop time", &pre_.crop_time);
    if (pre_.crop_time) {
        ImGui::Indent();
        input_us("Begin [us]##crop", pre_.t_begin);
        input_us("End [us]##crop", pre_.t_end);
        ImGui::Unindent();
    }
    ImGui::Checkbox("Crop region", &pre_.crop_roi);
    if (pre_.crop_roi) {
        ImGui::Indent();
        roi_inputs("croproi", pre_.roi, w, h);
        if (ImGui::SmallButton("Use evaluation ROI"))
            pre_.roi = params_.roi;
        ImGui::Unindent();
    }

    ImGui::BeginDisabled(!raw_ || job_running_);
    if (ImGui::Button("Apply pre-processing"))
        start_preprocess();
    ImGui::SameLine();
    ImGui::BeginDisabled(!preprocessed_);
    if (ImGui::Button("Revert to loaded events")) {
        set_data(raw_);
        preprocessed_ = false;
        pseudo_key_   = PseudoKey();
        bg_key_       = BgKey();
        log("Using the loaded events (pre-processing reverted)");
    }
    ImGui::EndDisabled();
    ImGui::EndDisabled();
    if (preprocessed_)
        ImGui::TextDisabled("Kept %.2f M / %.2f M events (%.1f %%), %zu hot pixels", pre_report_.out / 1e6,
                            pre_report_.in / 1e6, pre_report_.in ? 100.0 * pre_report_.out / pre_report_.in : 0.0,
                            pre_report_.hot_pixels);
    ImGui::PopID();
}

// ------------------------------------------------------------------------------------------- 3 pseudo images

void App::section_pseudo() {
    if (!header("3  Pseudo images"))
        return;
    ImGui::PushID("pseudo");
    PseudoSettings &s = pseudo_;
    ImGui::Checkbox("Follow evaluation window", &s.follow_evaluation);
    help("Shows the events of the current evaluation window (time step of the results, or SampleTimeStart). "
         "For sum of correlation / PIV: sub-volume or frame A, B, or both (A red, B cyan).");
    if (s.follow_evaluation && params_.method != Method::MotionComp) {
        ImGui::SameLine();
        ImGui::SetNextItemWidth(110);
        const char *frames[] = {"Frame A", "Frame B", "A + B"};
        ImGui::Combo("##frame", &s.piv_frame, frames, 3);
    }
    ImGui::BeginDisabled(s.follow_evaluation);
    input_us("Start [us]", s.t_us);
    input_us("Duration [us]", s.duration_us, 1);
    ImGui::EndDisabled();
    ImGui::SetNextItemWidth(kField * 1.6f);
    enum_combo("Image", s.mode, static_cast<int>(PseudoMode::Num), pseudo_name);
    ImGui::SetNextItemWidth(kField * 1.6f);
    polarity_combo("Polarity", s.polarity);
    ImGui::Checkbox("Remove isolated pixels", &s.despeckle);
    ImGui::SetNextItemWidth(kField * 1.6f);
    enum_combo("Colormap##pseudo", s.colormap, static_cast<int>(Colormap::Count), colormap_name);
    ImGui::Checkbox("Auto contrast", &s.auto_contrast);
    if (!s.auto_contrast) {
        ImGui::SetNextItemWidth(kField * 1.6f);
        ImGui::DragFloatRange2("Range", &s.lo, &s.hi, 0.05f, -1000, 1000, "%.2f");
    }
    ImGui::BeginDisabled(!data_);
    if (ImGui::Button("Save image...")) {
        const auto path = save_file_dialog("Save pseudo image", "PNG (as displayed)|*.png|TIFF (float values)|*.tif",
                                           data_->source.parent_path() /
                                               (data_->source.stem().u8string() + "_pseudo.png"),
                                           ".png");
        if (!path.empty()) {
            std::string ext = path.extension().string();
            std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return std::tolower(c); });
            if (save_pseudo_image(path, pseudo_img_, ext == ".png"))
                log("Pseudo image saved to " + to_utf8(path));
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Snap to pulses") && pulse_.period_us > 0) {
        const double k = std::round((s.t_us - pulse_.offset_us) / pulse_.period_us);
        s.t_us         = static_cast<int64_t>(std::llround(pulse_.offset_us + std::max(0.0, k) * pulse_.period_us));
        s.duration_us  = static_cast<int64_t>(std::llround(pulse_.period_us));
        s.follow_evaluation = false;
    }
    help("Start the window at the estimated pulse onset and make it one illumination period long "
         "(run 'Pulsed illumination > Analyse' first).");
    ImGui::EndDisabled();

    if (ImGui::TreeNode("Export series")) {
        input_int("Frames", series_count_, 1, 100000);
        input_us("Step [us]##series", series_step_us_, 1);
        ImGui::BeginDisabled(!data_ || job_running_);
        if (ImGui::Button("Export series...")) {
            const auto path = save_file_dialog("First file of the series (an index is appended)",
                                               "PNG (as displayed)|*.png|TIFF (float values)|*.tif",
                                               data_->source.parent_path() /
                                                   (data_->source.stem().u8string() + "_pseudo.png"),
                                               ".png");
            if (!path.empty())
                start_series_export(path);
        }
        ImGui::EndDisabled();
        ImGui::TreePop();
    }
    ImGui::PopID();
}

// ------------------------------------------------------------------------------------------- 4 pulsed light

void App::section_pulse() {
    if (!header("4  Pulsed illumination", false))
        return;
    ImGui::PushID("pulse");
    input_double("Frequency [Hz]", pulse_freq_, 10, "%.2f");
    input_int("Bin width [us]", pulse_bin_us_, 1, 100000, 5);
    input_int("Periods", pulse_periods_, 1, 1000000, 10);
    input_int("Start period", pulse_start_, 0, 1000000);
    ImGui::SetNextItemWidth(kField * 1.6f);
    polarity_combo("Polarity", pulse_pol_);
    ImGui::BeginDisabled(!data_);
    if (ImGui::Button("Analyse")) {
        pulse_ = pulse_histogram(*data_, pulse_freq_, pulse_bin_us_, pulse_periods_, pulse_start_, pulse_pol_);
        if (pulse_.rate.empty())
            log("Pulse analysis: no data (check frequency / bin width / periods)", true);
        else {
            char buf[200];
            std::snprintf(buf, sizeof(buf),
                          "Pulse analysis: period %.1f us, peak %.3f ev/us at %d us, onset (offset) %lld us",
                          pulse_.period_us, pulse_.max_rate, pulse_.max_bin * pulse_.bin_us,
                          static_cast<long long>(pulse_.offset_us));
            log(buf);
            select_tab_ = 2;
        }
    }
    ImGui::EndDisabled();
    if (pulse_.period_us > 0) {
        ImGui::SameLine();
        if (ImGui::Button("Use for PIV frames")) {
            params_.pulse_freq_hz   = pulse_.freq_hz;
            params_.pulse_offset_us = pulse_.offset_us;
            params_.piv_pulse_lock  = true;
            params_.method          = Method::Piv;
            log("PIV frames locked to the pulses (offset " + std::to_string(pulse_.offset_us) + " us)");
        }
        ImGui::TextDisabled("Onset: %lld us  |  peak %.3f ev/us  |  floor %.3f ev/us",
                            static_cast<long long>(pulse_.offset_us), pulse_.max_rate, pulse_.min_rate);
    }
    ImGui::PopID();
}

// ------------------------------------------------------------------------------------------- 5 evaluation

void App::section_evaluation() {
    if (!header("5  Evaluation (correlation)"))
        return;
    ImGui::PushID("eval");
    Params &p = params_;
    const int w = data_ ? data_->width : 1280, h = data_ ? data_->height : 720;

    ImGui::SetNextItemWidth(kField * 1.8f);
    enum_combo("Method", p.method, static_cast<int>(Method::Count), method_name);
    help("Motion compensation: warps the events of each sample with candidate velocities and maximizes the contrast "
         "of the warped image.\nSum of correlation: correlates two space-time sub-volumes separated by TimeOffset, "
         "slice by slice, and sums the correlation planes.\nPIV: correlates interrogation windows of two pseudo "
         "images (frames A / B).");

    if (ImGui::TreeNodeEx("Time sampling", ImGuiTreeNodeFlags_DefaultOpen)) {
        input_us("SampleTimeStart", p.t_start);
        input_us("SampleDuration", p.t_sample, 1);
        ImGui::Checkbox("Single time step", &p.single_time_step);
        if (!p.single_time_step) {
            input_us("SampleTimeEnd", p.t_end);
            input_us("SampleTimeStep", p.t_step, 1);
        }
        ImGui::TreePop();
    }
    if (ImGui::TreeNodeEx("Spatial sampling", ImGuiTreeNodeFlags_DefaultOpen)) {
        input_int("SampleWidth [px]", p.sample_w, 4, w, 2);
        input_int("SampleHeight [px]", p.sample_h, 4, h, 2);
        input_int("SampleStepX [px]", p.step_x, 1, w, 2);
        input_int("SampleStepY [px]", p.step_y, 1, h, 2);
        ImGui::SetNextItemWidth(kField * 1.6f);
        polarity_combo("EventPolarity", p.polarity);
        ImGui::Checkbox("Region of interest", &p.roi_enabled);
        if (p.roi_enabled) {
            ImGui::Indent();
            roi_inputs("roi", p.roi, w, h);
            if (ImGui::Checkbox("Draw on the image", &drawing_roi_) && drawing_roi_)
                select_tab_ = 0;
            ImGui::SameLine();
            if (ImGui::SmallButton("Full sensor"))
                p.roi = {0, 0, w, h};
            ImGui::Unindent();
        }
        ImGui::Checkbox("Show sampling grid", &show_grid_);
        ImGui::TreePop();
    }

    switch (p.method) {
    case Method::MotionComp:
        if (ImGui::TreeNodeEx("Motion compensation", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::TextDisabled("Velocity scan range [px/ms]");
            double vx[2] = {p.vx_min, p.vx_max}, vy[2] = {p.vy_min, p.vy_max};
            ImGui::SetNextItemWidth(kField * 1.6f);
            if (ImGui::InputScalarN("Vx min / max", ImGuiDataType_Double, vx, 2, nullptr, nullptr, "%.3g")) {
                p.vx_min = vx[0];
                p.vx_max = vx[1];
            }
            ImGui::SetNextItemWidth(kField * 1.6f);
            if (ImGui::InputScalarN("Vy min / max", ImGuiDataType_Double, vy, 2, nullptr, nullptr, "%.3g")) {
                p.vy_min = vy[0];
                p.vy_max = vy[1];
            }
            double res[2] = {p.res_x, p.res_y};
            ImGui::SetNextItemWidth(kField * 1.6f);
            if (ImGui::InputScalarN("Resolution x / y", ImGuiDataType_Double, res, 2, nullptr, nullptr, "%.3g")) {
                p.res_x = std::max(1e-3, res[0]);
                p.res_y = std::max(1e-3, res[1]);
            }
            const size_t nvx = velocity_candidates(p.vx_min, p.vx_max, p.res_x).size();
            const size_t nvy = velocity_candidates(p.vy_min, p.vy_max, p.res_y).size();
            ImGui::TextDisabled("%zu x %zu = %zu candidates per sample", nvx, nvy, nvx * nvy);
            ImGui::SetNextItemWidth(kField * 1.8f);
            enum_combo("ObjectiveFxn", p.objective, static_cast<int>(Objective::Count), objective_name);
            ImGui::SetNextItemWidth(kField * 1.8f);
            enum_combo("Interpolation", p.interpolation, static_cast<int>(Interpolation::Count), interpolation_name);
            help("Bilinear: correct bilinear weights. Bilinear (Python-compatible) reproduces ebiv_utils.py, which "
                 "gives the floor pixel the weight fx*fy instead of (1-fx)(1-fy); use it to compare with earlier "
                 "Python results.");
            ImGui::TreePop();
        }
        break;
    case Method::CorrSum:
        if (ImGui::TreeNodeEx("Sum of correlation", ImGuiTreeNodeFlags_DefaultOpen)) {
            input_us("TimeOffset [us]", p.t_sep, 1);
            help("Separation in time between the two sub-volumes (sampled at t - TimeOffset/2 and t + TimeOffset/2).");
            input_int("NT (slices)", p.nt, 1, 1000);
            ImGui::TreePop();
        }
        break;
    default:
        if (ImGui::TreeNodeEx("PIV", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::SetNextItemWidth(kField * 1.6f);
            enum_combo("Frame image", p.piv_image, static_cast<int>(PseudoMode::Num), pseudo_name);
            ImGui::Checkbox("Lock frames to the light pulses", &p.piv_pulse_lock);
            if (p.piv_pulse_lock) {
                input_double("Pulse frequency [Hz]", p.pulse_freq_hz, 10, "%.2f");
                input_us("Pulse onset [us]", p.pulse_offset_us);
                input_int("Frame separation [periods]", p.piv_pulse_separation, 1, 1000);
            } else {
                input_us("PIV_Delay [us]", p.piv_delay, 1);
                input_us("PIV_Exposure [us]", p.piv_exposure, 1);
            }
            Interval a, b;
            piv_frames(p, p.t_start, a, b);
            ImGui::TextDisabled("Frames at t0: A [%lld, %lld)  B [%lld, %lld) us", static_cast<long long>(a.t1),
                                static_cast<long long>(a.t2), static_cast<long long>(b.t1), static_cast<long long>(b.t2));
            ImGui::TreePop();
        }
        break;
    }
    ImGui::Checkbox("Gaussian peak fit", &p.gauss_peak_fit);
    help("3-point fit of the logarithm of the values (parabolic fit otherwise / when a value is <= 0).");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(70);
    ImGui::InputInt("Threads", &p.threads, 0);
    p.threads = std::clamp(p.threads, 0, 64);
    help("0 = all cores");

    const Grid g = sampling_grid(p, w, h);
    const size_t steps = time_steps(p).size();
    ImGui::TextDisabled("Grid %d x %d = %d samples, %zu time step(s)", g.nx, g.ny, g.nx * g.ny, steps);

    ImGui::BeginDisabled(!data_ || job_running_);
    if (ImGui::Button("Run EBIV  (F5)", ImVec2(-1, 0)))
        start_processing();
    ImGui::EndDisabled();
    if (ImGui::Button("Load .cfg...")) {
        const auto path = open_file_dialog("Load EBIV parameters", "EBIV parameters|*.cfg;*.ini|All files|*.*");
        if (!path.empty())
            load_config(path);
    }
    ImGui::SameLine();
    if (ImGui::Button("Save .cfg...")) {
        const auto path = save_file_dialog("Save EBIV parameters", "EBIV parameters|*.cfg|All files|*.*",
                                           cfg_path_.empty() ? std::filesystem::path("ebiv_params.cfg") : cfg_path_,
                                           ".cfg");
        std::string error;
        if (!path.empty()) {
            if (save_cfg(path, p, &error)) {
                cfg_path_ = path;
                log("Parameters saved to " + to_utf8(path));
            } else
                log(error, true);
        }
    }
    if (!cfg_path_.empty()) {
        ImGui::SameLine();
        ImGui::TextDisabled("%s", to_utf8(cfg_path_.filename()).c_str());
    }
    ImGui::PopID();
}

// ------------------------------------------------------------------------------------------- 6 post-processing

void App::section_postprocess() {
    if (!header("6  Post-processing"))
        return;
    ImGui::PushID("post");
    PostParams &p = post_;
    bool changed  = ImGui::Checkbox("Outlier detection", &p.enabled);
    if (p.enabled) {
        changed |= input_double("Median threshold", p.median_threshold, 0.5, "%.2f");
        help("Normalized median test on vx, vy and |v| over the 8 neighbours (velmap_utils.DetectOutliers); a vector "
             "is an outlier when two of the three tests fail. 0 disables it.");
        changed |= input_double("Noise floor [px/ms]", p.noise_floor, 0.01, "%.3f");
        changed |= input_double("Max. |v| [px/ms]", p.max_magnitude, 0.5, "%.3g");
        help("0 = off");
        changed |= input_double("Min. events", p.min_events, 10, "%.0f");
        changed |= input_double("Min. peak value", p.min_peak, 0.01, "%.3g");
        changed |= ImGui::Checkbox("Flag samples without estimate", &p.flag_empty);
        changed |= ImGui::Checkbox("Replace outliers", &p.replace);
        help("Replaced by the mean of the valid neighbours, growing inward (the Python scripts interpolate with "
             "scipy griddata).");
    }
    if (changed) {
        p.median_threshold = std::max(0.0, p.median_threshold);
        revalidate();
    }
    if (result_ && step_ < static_cast<int>(result_->fields.size())) {
        const Field &f = result_->fields[step_];
        ImGui::TextDisabled("Time step %d: %d outliers of %zu (%.1f %%), %d replaced", step_, f.post.outliers, f.v.size(),
                            f.v.empty() ? 0.0 : 100.0 * f.post.outliers / f.v.size(), f.post.replaced);
    }
    ImGui::PopID();
}

// ------------------------------------------------------------------------------------------- 7 display

void App::section_display() {
    if (!header("7  Display"))
        return;
    ImGui::PushID("disp");
    DisplaySettings &d = display_;
    ImGui::SetNextItemWidth(kField * 1.6f);
    enum_combo("Unit", d.unit, static_cast<int>(VelocityUnit::Count), unit_name);
    input_double("Magnification [px/mm]", params_.magnification, 1, "%.4g");
    params_.magnification = std::max(0.0, params_.magnification);
    if (d.unit == VelocityUnit::MetersPerSecond && params_.magnification <= 0)
        ImGui::TextColored(ImVec4(1, 0.7f, 0.3f, 1), "Set a magnification to show m/s");
    ImGui::SetNextItemWidth(kField * 1.6f);
    enum_combo("Background", d.background, static_cast<int>(Background::Count), background_name);
    ImGui::SetNextItemWidth(kField * 1.6f);
    enum_combo("Scalar", d.scalar, static_cast<int>(Scalar::Count), scalar_name);
    ImGui::SetNextItemWidth(kField * 1.6f);
    enum_combo("Colormap", d.colormap, static_cast<int>(Colormap::Count), colormap_name);
    ImGui::Checkbox("Auto range", &d.auto_range);
    if (!d.auto_range) {
        ImGui::SetNextItemWidth(kField * 1.6f);
        ImGui::InputFloat2("Min / max", &d.lo);
    }
    ImGui::SetNextItemWidth(kField);
    ImGui::SliderFloat("Map opacity", &d.map_alpha, 0.f, 1.f, "%.2f");
    ImGui::Checkbox("Vectors", &d.show_vectors);
    if (d.show_vectors) {
        ImGui::Indent();
        ImGui::SetNextItemWidth(kField);
        enum_combo("Color", d.vector_color, static_cast<int>(VectorColor::Count), vector_color_name);
        ImGui::Checkbox("Auto scale", &d.auto_scale);
        ImGui::SameLine();
        ImGui::BeginDisabled(d.auto_scale);
        ImGui::SetNextItemWidth(kField);
        ImGui::DragFloat("Scale", &d.vector_scale, 0.02f, 0.01f, 1000.f, "%.2f");
        ImGui::EndDisabled();
        help("Arrow length = displacement over the method's time base [px] x scale. Auto: the 95th percentile spans "
             "about one grid step.");
        ImGui::SetNextItemWidth(kField);
        ImGui::SliderFloat("Width", &d.vector_width, 0.5f, 4.f, "%.1f");
        ImGui::Checkbox("Outliers in red", &d.show_outliers);
        ImGui::SameLine();
        ImGui::Checkbox("Samples without estimate", &d.show_invalid);
        ImGui::Unindent();
    }
    ImGui::PopID();
}

// ------------------------------------------------------------------------------------------- 8 export

void App::section_export() {
    if (!header("8  Export", false))
        return;
    ImGui::PushID("export");
    ImGui::BeginDisabled(!result_);
    ImGui::Checkbox("Post-processed velocities in .npz", &show_validated_);
    help("The .npz has the layout of ebiv_process.py / ebiv_run.py: (nt, ny, nx, 6) = x, imgH - y, vx, vy, peak, "
         "events. Unchecked: raw estimates (the Python scripts validate when plotting).");
    const auto stem = [&](const char *ext) {
        const auto src = data_ ? data_->source : std::filesystem::path("ebiv");
        return src.parent_path() /
               (src.stem().u8string() + "_ebiv_" + (result_ ? to_string(result_->params.method) : "") + ext);
    };
    if (ImGui::Button("Save .npz...")) {
        const auto path = save_file_dialog("Export velocity data", "NumPy archive|*.npz", stem(".npz"), ".npz");
        std::string error;
        if (!path.empty()) {
            if (save_npz(path, *result_, show_validated_, &error))
                log("Velocity data saved to " + to_utf8(path));
            else
                log(error, true);
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Save .csv...")) {
        const auto path = save_file_dialog("Export velocity data", "CSV|*.csv", stem(".csv"), ".csv");
        std::string error;
        if (!path.empty()) {
            if (save_csv(path, *result_, &error))
                log("Velocity data saved to " + to_utf8(path));
            else
                log(error, true);
        }
    }
    ImGui::EndDisabled();
    ImGui::BeginDisabled(!data_);
    if (ImGui::Button("Save view as PNG...")) {
        const auto path = save_file_dialog("Save view", "PNG|*.png", stem(".png"), ".png");
        if (!path.empty()) {
            capture_.path    = path;
            capture_.min     = view_min_;
            capture_.max     = view_max_;
            capture_.pending = true;
        }
    }
    help("Saves the Viewer content (image, vectors, colorbar) as shown on screen.");
    ImGui::EndDisabled();
    ImGui::PopID();
}

} // namespace ebiv::app
