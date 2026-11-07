#include "ebiv_app/app.h"

#include <algorithm>
#include <cstdio>
#include <ctime>

#include <imgui_internal.h>

#include "app/platform.h"
#include "ebiv/export.h"

namespace ebiv::app {

using evs::app::from_utf8;
using evs::app::open_file_dialog;
using evs::app::save_file_dialog;
using evs::app::to_utf8;

namespace {

int64_t timeline_bin(const EventSet &set) {
    // at most ~200k bins, at least 50 us
    const int64_t d = set.duration_us();
    int64_t bin     = 50;
    while (d / bin > 200000)
        bin *= 2;
    return bin;
}

std::string with_suffix(const std::filesystem::path &source, const std::string &suffix) {
    if (source.empty())
        return suffix;
    return to_utf8(source.parent_path() / (source.stem().u8string() + suffix));
}

} // namespace

bool App::PseudoKey::operator==(const PseudoKey &o) const {
    return data == o.data && t1 == o.t1 && t2 == o.t2 && t1b == o.t1b && t2b == o.t2b && pol == o.pol &&
           mode == o.mode && frame == o.frame && cm == o.cm && despeckle == o.despeckle &&
           auto_contrast == o.auto_contrast && overlay == o.overlay && lo == o.lo && hi == o.hi;
}

App::App() {
    log("EBIV Studio ready. Open a recording (.raw, .hdf5, .dat or EBIV .evt) with File > Open or drop it here.");
}

App::~App() { shutdown(); }

void App::shutdown() {
    if (job_thread_.joinable()) {
        if (job_cancel_)
            job_cancel_();
        job_thread_.join();
    }
    job_running_ = false;
}

void App::log(const std::string &message, bool error) {
    const std::time_t now = std::time(nullptr);
    std::tm tm{};
    localtime_s(&tm, &now);
    char t[16];
    std::snprintf(t, sizeof(t), "%02d:%02d:%02d", tm.tm_hour, tm.tm_min, tm.tm_sec);
    log_.push_back({t, message, error});
    if (log_.size() > 2000)
        log_.erase(log_.begin(), log_.begin() + 500);
}

// ------------------------------------------------------------------------------------------------- jobs

void App::start_job(const std::string &label, std::function<void()> work, std::function<void()> finish,
                    std::function<double()> fraction, std::function<void()> cancel) {
    if (job_running_)
        return;
    if (job_thread_.joinable())
        job_thread_.join();
    job_label_    = label;
    job_finish_   = std::move(finish);
    job_fraction_ = std::move(fraction);
    job_cancel_   = std::move(cancel);
    job_start_    = std::chrono::steady_clock::now();
    job_done_     = false;
    job_running_  = true;
    job_thread_   = std::thread([this, work = std::move(work)] {
        work();
        job_done_ = true;
    });
}

void App::poll_job() {
    if (!job_running_ || !job_done_)
        return;
    job_thread_.join();
    job_running_ = false;
    auto finish  = std::move(job_finish_);
    job_finish_  = nullptr;
    job_fraction_ = nullptr;
    job_cancel_   = nullptr;
    if (finish)
        finish();
}

void App::open_file(const std::filesystem::path &path) {
    std::string ext = path.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return std::tolower(c); });
    if (ext == ".cfg" || ext == ".ini") {
        load_config(path);
        return;
    }
    start_load(path);
}

void App::start_load(const std::filesystem::path &path) {
    if (job_running_) {
        log("Wait for the current job to finish (or cancel it) before opening another recording", true);
        return;
    }
    auto set      = std::make_shared<EventSet>();
    auto progress = std::make_shared<LoadProgress>();
    auto error    = std::make_shared<std::string>();
    auto ok       = std::make_shared<bool>(false);
    auto timeline = std::make_shared<std::vector<float>>();
    auto bin      = std::make_shared<int64_t>(1000);
    log("Loading " + to_utf8(path) + " ...");
    start_job(
        "Loading " + to_utf8(path.filename()),
        [=] {
            *ok = load_events(path, *set, error.get(), progress.get());
            if (*ok) {
                *bin      = timeline_bin(*set);
                *timeline = rate_timeline(*set, *bin);
            }
        },
        [=] {
            const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - job_start_).count();
            if (!*ok) {
                log("Loading failed: " + *error, true);
                return;
            }
            if (!error->empty())
                log("Warning: " + *error, true);
            raw_          = set;
            data_         = set;
            timeline_     = std::move(*timeline);
            timeline_bin_us_ = *bin;
            preprocessed_ = false;
            pre_report_   = PreprocessReport();
            result_.reset();
            detail_.reset();
            has_selection_ = false;
            tl_t0_ = 0;
            tl_t1_ = static_cast<double>(set->duration_us());
            pseudo_key_ = PseudoKey();
            bg_key_     = BgKey();
            if (pre_.roi.w == 0) {
                pre_.roi = {0, 0, set->width, set->height};
            }
            if (params_.roi.w == 0)
                params_.roi = {0, 0, set->width, set->height};
            char buf[256];
            std::snprintf(buf, sizeof(buf), "Loaded %s: %dx%d, %s, %.2f M events, %.3f s, %zu triggers (%.1f s, %s)",
                          to_utf8(set->source.filename()).c_str(), set->width, set->height, set->format.c_str(),
                          set->events.size() / 1e6, set->duration_us() / 1e6, set->triggers.size(), s,
                          evs::app::human_bytes(static_cast<double>(set->memory_bytes())).c_str());
            log(buf);
            const auto cfg = set->source.parent_path() / "ebiv_params.cfg";
            if (cfg_path_.empty() && std::filesystem::exists(cfg))
                log("Tip: " + to_utf8(cfg) + " found next to the recording (File > Load parameters)");
            select_tab_ = 0;
            if (run_after_load_) {
                run_after_load_ = false;
                start_processing();
            }
        },
        [=] { return progress->fraction.load(); }, [=] { progress->cancel = true; });
}

void App::set_data(std::shared_ptr<const EventSet> data) {
    data_            = std::move(data);
    timeline_bin_us_ = timeline_bin(*data_);
    timeline_        = rate_timeline(*data_, timeline_bin_us_);
}

void App::start_preprocess() {
    if (!raw_ || job_running_)
        return;
    if (!any_stage(pre_)) {
        data_        = raw_;
        set_data(raw_);
        preprocessed_ = false;
        log("No pre-processing stage enabled: using the loaded events");
        return;
    }
    auto in       = raw_;
    auto out      = std::make_shared<EventSet>();
    auto params   = pre_;
    auto report   = std::make_shared<PreprocessReport>();
    auto progress = std::make_shared<PreprocessProgress>();
    auto error    = std::make_shared<std::string>();
    auto ok       = std::make_shared<bool>(false);
    auto timeline = std::make_shared<std::vector<float>>();
    auto bin      = std::make_shared<int64_t>(1000);
    start_job(
        "Pre-processing",
        [=] {
            *ok = preprocess(*in, params, *out, *report, error.get(), progress.get());
            if (*ok) {
                *bin      = timeline_bin(*out);
                *timeline = rate_timeline(*out, *bin);
            }
        },
        [=] {
            if (!*ok) {
                log("Pre-processing failed: " + *error, true);
                return;
            }
            data_            = out;
            timeline_        = std::move(*timeline);
            timeline_bin_us_ = *bin;
            preprocessed_    = true;
            pre_report_      = *report;
            pseudo_key_      = PseudoKey();
            bg_key_          = BgKey();
            char buf[200];
            std::snprintf(buf, sizeof(buf), "Pre-processing: kept %.2f M of %.2f M events (%.1f %%), %zu hot pixels, %.1f s",
                          report->out / 1e6, report->in / 1e6, report->in ? 100.0 * report->out / report->in : 0.0,
                          report->hot_pixels, report->seconds);
            log(buf);
        },
        [=] { return progress->fraction.load(); }, [=] { progress->cancel = true; });
}

void App::start_processing() {
    if (!data_ || job_running_)
        return;
    const Grid g = sampling_grid(params_, data_->width, data_->height);
    if (g.nx <= 0 || g.ny <= 0) {
        log("The sampling grid is empty: check the sample size and the ROI", true);
        return;
    }
    if (params_.method == Method::MotionComp &&
        (velocity_candidates(params_.vx_min, params_.vx_max, params_.res_x).size() < 2 ||
         velocity_candidates(params_.vy_min, params_.vy_max, params_.res_y).size() < 2)) {
        log("The velocity scan range needs at least 2 candidates in vx and vy", true);
        return;
    }
    auto data     = data_;
    auto params   = params_;
    auto progress = std::make_shared<ProcessProgress>();
    auto result   = std::make_shared<Result>();
    const size_t steps = time_steps(params).size();
    log(std::string("Running ") + display_name(params.method) + " on " + std::to_string(g.nx) + " x " +
        std::to_string(g.ny) + " samples x " + std::to_string(steps) + " time step(s) ...");
    start_job(
        std::string("EBIV: ") + display_name(params.method),
        [=] { *result = run_ebiv(*data, params, progress.get()); },
        [=] {
            if (progress->cancel) {
                log("Processing cancelled (" + std::to_string(result->fields.size()) + " time steps kept)");
                if (result->fields.empty())
                    return;
            }
            result_ = result;
            step_   = 0;
            has_selection_ = false;
            detail_.reset();
            bg_key_ = BgKey();
            revalidate();
            select_tab_ = 1;
            inspect(result->grid.nx / 2, result->grid.ny / 2); // something to look at in the Inspector
            char buf[200];
            std::snprintf(buf, sizeof(buf), "Done: %llu samples in %.2f s (%.0f samples/s)",
                          static_cast<unsigned long long>(result->samples), result->seconds,
                          result->seconds > 0 ? result->samples / result->seconds : 0.0);
            log(buf);
            int valid = 0;
            for (const auto &v : result->fields.front().v)
                valid += v.valid;
            log("First time step: " + std::to_string(valid) + " of " + std::to_string(result->fields.front().v.size()) +
                " samples with an estimate, " + std::to_string(result->fields.front().post.outliers) + " outliers");
        },
        [=] { return progress->fraction.load(); }, [=] { progress->cancel = true; });
}

void App::revalidate() {
    if (!result_)
        return;
    for (Field &f : result_->fields)
        validate(f, result_->grid, post_);
}

bool App::load_config(const std::filesystem::path &path) {
    std::string error;
    Params p = params_;
    if (!ebiv::load_cfg(path, p, &error)) {
        log(error, true);
        return false;
    }
    if (data_ && (p.roi.w == 0 || p.roi.h == 0))
        p.roi = {0, 0, data_->width, data_->height};
    params_   = p;
    cfg_path_ = path;
    log("Loaded parameters from " + to_utf8(path));
    return true;
}

void App::capture_finished(bool ok, const std::string &error) {
    if (ok)
        log("View saved to " + to_utf8(capture_.path));
    else
        log("Saving the view failed: " + error, true);
    capture_.pending = false;
}

// ------------------------------------------------------------------------------------------------- UI

void App::frame() {
    poll_job();
    const ImGuiIO &io = ImGui::GetIO();
    if (!io.WantTextInput) {
        if (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_O)) {
            const auto path = open_file_dialog("Open recording",
                                               "Event recordings|*.raw;*.hdf5;*.h5;*.dat;*.evt|All files|*.*");
            if (!path.empty())
                open_file(path);
        }
        if (ImGui::IsKeyPressed(ImGuiKey_F5) && !job_running_)
            start_processing();
        if (ImGui::IsKeyPressed(ImGuiKey_Space) && result_ && result_->fields.size() > 1)
            playing_ = !playing_;
    }
    if (playing_ && result_ && result_->fields.size() > 1) {
        const auto now = std::chrono::steady_clock::now();
        if (now - last_play_ > std::chrono::milliseconds(200)) {
            last_play_ = now;
            step_      = (step_ + 1) % static_cast<int>(result_->fields.size());
            if (has_selection_)
                inspect(sel_col_, sel_row_);
        }
    }

    draw_menu_bar();
    draw_dockspace();
    draw_pipeline_window();
    draw_viewer_window();
    draw_inspector_window();
    draw_log_window();
    if (show_about_)
        draw_about();
    if (show_imgui_demo_)
        ImGui::ShowDemoWindow(&show_imgui_demo_);
}

void App::draw_dockspace() {
    const ImGuiID dock_id = ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport());
    if (layout_initialized_)
        return;
    layout_initialized_ = true;
    ImGuiDockNode *node = ImGui::DockBuilderGetNode(dock_id);
    if (node && (node->IsSplitNode() || node->Windows.Size > 0))
        return; // layout restored from the .ini file

    ImGui::DockBuilderRemoveNode(dock_id);
    ImGui::DockBuilderAddNode(dock_id, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(dock_id, ImGui::GetMainViewport()->WorkSize);
    ImGuiID center = dock_id;
    const ImGuiID left   = ImGui::DockBuilderSplitNode(center, ImGuiDir_Left, 0.25f, nullptr, &center);
    const ImGuiID right  = ImGui::DockBuilderSplitNode(center, ImGuiDir_Right, 0.24f, nullptr, &center);
    const ImGuiID bottom = ImGui::DockBuilderSplitNode(center, ImGuiDir_Down, 0.14f, nullptr, &center);
    ImGui::DockBuilderDockWindow("Pipeline", left);
    ImGui::DockBuilderDockWindow("Inspector", right);
    ImGui::DockBuilderDockWindow("Log", bottom);
    ImGui::DockBuilderDockWindow("Viewer", center);
    ImGui::DockBuilderFinish(dock_id);
}

void App::draw_menu_bar() {
    if (!ImGui::BeginMainMenuBar())
        return;
    if (ImGui::BeginMenu("File")) {
        if (ImGui::MenuItem("Open recording...", "Ctrl+O", false, !job_running_)) {
            const auto path = open_file_dialog("Open recording",
                                               "Event recordings|*.raw;*.hdf5;*.h5;*.dat;*.evt|All files|*.*");
            if (!path.empty())
                open_file(path);
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Load parameters...")) {
            const auto path = open_file_dialog("Load EBIV parameters", "EBIV parameters|*.cfg;*.ini|All files|*.*");
            if (!path.empty())
                load_config(path);
        }
        if (ImGui::MenuItem("Save parameters...")) {
            const auto path = save_file_dialog("Save EBIV parameters", "EBIV parameters|*.cfg|All files|*.*",
                                               cfg_path_.empty() ? std::filesystem::path("ebiv_params.cfg") : cfg_path_,
                                               ".cfg");
            std::string error;
            if (!path.empty()) {
                if (save_cfg(path, params_, &error)) {
                    cfg_path_ = path;
                    log("Parameters saved to " + to_utf8(path));
                } else
                    log(error, true);
            }
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Export results (.npz)...", nullptr, false, result_ != nullptr)) {
            const auto path = save_file_dialog("Export velocity data", "NumPy archive|*.npz",
                                               from_utf8(with_suffix(data_ ? data_->source : std::filesystem::path(),
                                                                     std::string("_ebiv_") + to_string(result_->params.method) + ".npz")),
                                               ".npz");
            std::string error;
            if (!path.empty()) {
                if (save_npz(path, *result_, show_validated_, &error))
                    log("Velocity data saved to " + to_utf8(path));
                else
                    log(error, true);
            }
        }
        if (ImGui::MenuItem("Export results (.csv)...", nullptr, false, result_ != nullptr)) {
            const auto path = save_file_dialog("Export velocity data", "CSV|*.csv",
                                               from_utf8(with_suffix(data_ ? data_->source : std::filesystem::path(),
                                                                     std::string("_ebiv_") + to_string(result_->params.method) + ".csv")),
                                               ".csv");
            std::string error;
            if (!path.empty()) {
                if (save_csv(path, *result_, &error))
                    log("Velocity data saved to " + to_utf8(path));
                else
                    log(error, true);
            }
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Quit", "Alt+F4"))
            quit_requested_ = true;
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Process")) {
        if (ImGui::MenuItem("Apply pre-processing", nullptr, false, raw_ && !job_running_))
            start_preprocess();
        if (ImGui::MenuItem("Run EBIV", "F5", false, data_ && !job_running_))
            start_processing();
        if (ImGui::MenuItem("Cancel job", nullptr, false, job_running_.load()) && job_cancel_)
            job_cancel_();
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Help")) {
        ImGui::MenuItem("About EBIV Studio", nullptr, &show_about_);
        ImGui::MenuItem("ImGui demo", nullptr, &show_imgui_demo_);
        ImGui::EndMenu();
    }
    if (job_running_) {
        const double f = job_fraction_ ? job_fraction_() : 0;
        ImGui::SameLine(ImGui::GetWindowWidth() - 420);
        ImGui::TextUnformatted(job_label_.c_str());
        ImGui::SameLine();
        ImGui::ProgressBar(static_cast<float>(f), ImVec2(160, 0));
    } else if (data_) {
        char buf[160];
        std::snprintf(buf, sizeof(buf), "%s  |  %.2f M events  |  %.3f s", to_utf8(data_->source.filename()).c_str(),
                      data_->events.size() / 1e6, data_->duration_us() / 1e6);
        ImGui::SameLine(ImGui::GetWindowWidth() - ImGui::CalcTextSize(buf).x - 20);
        ImGui::TextDisabled("%s", buf);
    }
    ImGui::EndMainMenuBar();
}

void App::draw_log_window() {
    if (!ImGui::Begin("Log")) {
        ImGui::End();
        return;
    }
    if (ImGui::SmallButton("Clear"))
        log_.clear();
    ImGui::Separator();
    ImGui::BeginChild("log_lines");
    for (const auto &e : log_) {
        ImGui::TextDisabled("%s", e.time.c_str());
        ImGui::SameLine();
        if (e.error)
            ImGui::TextColored(ImVec4(1.f, 0.45f, 0.4f, 1.f), "%s", e.text.c_str());
        else
            ImGui::TextUnformatted(e.text.c_str());
    }
    if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 5)
        ImGui::SetScrollHereY(1.f);
    ImGui::EndChild();
    ImGui::End();
}

void App::draw_about() {
    ImGui::SetNextWindowSize(ImVec2(560, 0), ImGuiCond_Appearing);
    if (ImGui::Begin("About EBIV Studio", &show_about_, ImGuiWindowFlags_NoDocking)) {
        ImGui::TextUnformatted("EBIV Studio - event-based imaging velocimetry");
        ImGui::Separator();
        ImGui::TextWrapped(
            "C++ interface to the EBIV processing of C. Willert (DLR): motion compensation (contrast maximization), "
            "sum of correlation over space-time sub-volumes and PIV on pseudo-image pairs, with pre-processing "
            "(EventStudio software filters, hot pixels), pseudo-image reconstruction, pulsed-illumination analysis, "
            "outlier detection and result display / export (.npz compatible with the Python plot scripts).");
        ImGui::Spacing();
        ImGui::TextWrapped("Viewer: mouse wheel zooms, right / middle drag pans, double click resets. Results: hover "
                           "for values, click a vector to inspect its sample. F5 runs the evaluation, Space plays a "
                           "time series.");
    }
    ImGui::End();
}

} // namespace ebiv::app
