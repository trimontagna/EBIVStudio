#pragma once

#include <atomic>
#include <chrono>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <imgui.h>

#include "ebiv/events.h"
#include "ebiv/params.h"
#include "ebiv/postprocess.h"
#include "ebiv/preprocess.h"
#include "ebiv/processor.h"
#include "ebiv/pulse.h"
#include "ebiv/sampling.h"
#include "ebiv_app/render.h"

namespace ebiv::app {

// Pseudo-image view settings (Pseudo image tab).
struct PseudoSettings {
    int64_t t_us        = 10000;
    int64_t duration_us = 10000;
    Polarity polarity   = Polarity::Positive;
    PseudoMode mode     = PseudoMode::Count;
    bool despeckle      = false;
    bool follow_evaluation = false; // window = current evaluation window (frames A / B for PIV)
    int piv_frame       = 2;        // follow + PIV: 0 frame A, 1 frame B, 2 overlay A (red) / B (cyan)
    Colormap colormap   = Colormap::Gray;
    bool auto_contrast  = true;
    float lo = 0, hi = 4;
};

enum class Background { None, PseudoImage, ScalarMap, Count };
enum class VelocityUnit { PxPerMs, DisplacementPx, MetersPerSecond, Count };
enum class VectorColor { Scalar, White, Black, Count };

struct DisplaySettings {
    Scalar scalar         = Scalar::Magnitude;
    Colormap colormap     = Colormap::Jet;
    bool auto_range       = true;
    float lo = 0, hi = 1;
    Background background = Background::ScalarMap;
    float map_alpha       = 0.85f;
    bool show_vectors     = true;
    VectorColor vector_color = VectorColor::White;
    float vector_scale    = 1.f;  // arrow length = displacement [px] * scale
    bool auto_scale       = true;
    float vector_width    = 1.5f;
    bool show_outliers    = true;
    bool show_invalid     = false;
    VelocityUnit unit     = VelocityUnit::PxPerMs;
};

// The whole application state; UI code lives in app.cpp (jobs, menus, layout), panels.cpp (Pipeline window)
// and viewer.cpp (Viewer, Inspector).
class App {
public:
    App();
    ~App();

    void frame();
    void open_file(const std::filesystem::path &path);
    bool load_config(const std::filesystem::path &path);
    void shutdown();
    bool quit_requested() const { return quit_requested_; }
    // Starts the evaluation as soon as the next recording is loaded (command line --run).
    void run_after_load() { run_after_load_ = true; }

    // "Save view as PNG": the main loop reads the framebuffer region after rendering the frame.
    struct Capture {
        bool pending = false;
        ImVec2 min, max; // screen coordinates
        std::filesystem::path path;
    };
    Capture &capture() { return capture_; }
    void capture_finished(bool ok, const std::string &error);

private:
    // ---- jobs (app.cpp): one background job at a time
    void start_job(const std::string &label, std::function<void()> work, std::function<void()> finish,
                   std::function<double()> fraction, std::function<void()> cancel);
    void poll_job();
    bool job_running() const { return job_running_; }

    void start_load(const std::filesystem::path &path);
    void start_preprocess();
    void start_processing();
    void start_series_export(const std::filesystem::path &first);
    void set_data(std::shared_ptr<const EventSet> data);
    void revalidate();

    // ---- UI (app.cpp)
    void draw_dockspace();
    void draw_menu_bar();
    void draw_log_window();
    void draw_about();
    void log(const std::string &message, bool error = false);

    // ---- Pipeline window (panels.cpp)
    void draw_pipeline_window();
    void section_recording();
    void section_preprocess();
    void section_pseudo();
    void section_pulse();
    void section_evaluation();
    void section_postprocess();
    void section_display();
    void section_export();
    void draw_job_progress();

    // ---- Viewer / Inspector (viewer.cpp)
    void draw_viewer_window();
    void tab_pseudo();
    void tab_results();
    void tab_pulse();
    void draw_timeline(float height);
    bool begin_image_view(const char *id, ImVec2 size, ViewTransform &view, bool &hovered);
    void draw_sampling_overlay(ImDrawList *dl, const ViewTransform &view);
    void draw_inspector_window();
    void inspect(int col, int row);
    void update_pseudo_image();
    void update_result_background();
    void current_window(int64_t &t1, int64_t &t2) const;
    void evaluation_windows(int64_t t0, Interval &a, Interval &b) const;
    float unit_factor(const Result &r, float velocity_px_ms) const;
    const char *unit_label() const;
    bool save_pseudo_image(const std::filesystem::path &path, const Image &img, bool colored);

    // ---- data
    std::shared_ptr<const EventSet> raw_;  // as loaded
    std::shared_ptr<const EventSet> data_; // after pre-processing (== raw_ when none applied)
    std::vector<float> timeline_;          // event rate of data_ [ev/us]
    int64_t timeline_bin_us_ = 1000;
    std::string load_error_;

    // ---- parameters
    Params params_;
    PreprocessParams pre_;
    PreprocessReport pre_report_;
    bool preprocessed_ = false;
    PostParams post_;
    std::filesystem::path cfg_path_;

    // ---- results
    std::shared_ptr<Result> result_;
    int step_ = 0;
    bool playing_ = false;
    std::chrono::steady_clock::time_point last_play_;
    bool show_validated_ = true; // npz export: post-processed values

    // ---- pseudo images
    PseudoSettings pseudo_;
    Image pseudo_img_, pseudo_img_b_;
    std::vector<uint8_t> rgba_;
    Texture pseudo_tex_;
    struct PseudoKey {
        const EventSet *data = nullptr;
        int64_t t1 = 0, t2 = 0, t1b = 0, t2b = 0;
        int pol = 0, mode = 0, frame = 0, cm = 0;
        bool despeckle = false, auto_contrast = true, overlay = false;
        float lo = 0, hi = 0;
        bool operator==(const PseudoKey &o) const;
    } pseudo_key_;
    int series_count_   = 20;
    int64_t series_step_us_ = 5000;

    // ---- results background
    Texture result_bg_tex_;
    struct BgKey {
        const EventSet *data = nullptr;
        const Result *result = nullptr;
        int step = -1, cm = 0;
        bool operator==(const BgKey &o) const {
            return data == o.data && result == o.result && step == o.step && cm == o.cm;
        }
    } bg_key_;

    // ---- view
    DisplaySettings display_;
    float zoom_ = 1.f;
    ImVec2 pan_{0, 0};
    bool drawing_roi_ = false;
    bool roi_drag_    = false;
    ImVec2 roi_start_{0, 0};
    bool show_grid_   = true;
    int viewer_tab_   = 0;   // 0 pseudo, 1 results, 2 pulse (to select a tab programmatically)
    int select_tab_   = -1;
    ImVec2 view_min_{0, 0}, view_max_{0, 0}; // last drawn viewer content (PNG capture)
    // timeline
    double tl_t0_ = 0, tl_t1_ = 0; // visible time range [us]

    // ---- inspector
    bool has_selection_ = false;
    int sel_col_ = 0, sel_row_ = 0;
    std::unique_ptr<SampleDetail> detail_;
    Texture insp_tex_[3];

    // ---- pulsed illumination
    double pulse_freq_   = 200;
    int pulse_bin_us_    = 50;
    int pulse_periods_   = 100;
    int pulse_start_     = 0;
    Polarity pulse_pol_  = Polarity::Positive;
    PulseHistogram pulse_;

    // ---- job
    std::thread job_thread_;
    std::atomic<bool> job_running_{false};
    std::atomic<bool> job_done_{false};
    std::string job_label_;
    std::function<void()> job_finish_;
    std::function<double()> job_fraction_;
    std::function<void()> job_cancel_;
    std::chrono::steady_clock::time_point job_start_;

    // ---- misc
    struct LogEntry {
        std::string time, text;
        bool error;
    };
    std::vector<LogEntry> log_;
    Capture capture_;
    bool show_about_         = false;
    bool show_imgui_demo_    = false;
    bool quit_requested_     = false;
    bool layout_initialized_ = false;
    bool run_after_load_     = false;
};

} // namespace ebiv::app
