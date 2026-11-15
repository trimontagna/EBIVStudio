// EBIV Studio: event-based imaging velocimetry (motion compensation, sum of correlation, PIV on pseudo images).

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <vector>

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "app/launch_console.h"
#include "ebiv_app/app.h"

namespace {

std::vector<std::filesystem::path> g_dropped_files;

void glfw_error_callback(int error, const char *description) {
    std::fprintf(stderr, "GLFW error %d: %s\n", error, description);
}

void drop_callback(GLFWwindow *, int count, const char **paths) {
    for (int i = 0; i < count; ++i)
        g_dropped_files.push_back(std::filesystem::u8path(paths[i]));
}

void setup_style(float scale) {
    ImGui::StyleColorsDark();
    ImGuiStyle &style       = ImGui::GetStyle();
    style.WindowRounding    = 4.f;
    style.FrameRounding     = 3.f;
    style.GrabRounding      = 3.f;
    style.TabRounding       = 3.f;
    style.ScrollbarRounding = 6.f;
    style.FramePadding      = ImVec2(6, 4);
    style.ItemSpacing       = ImVec2(8, 6);
    ImVec4 *c                 = style.Colors;
    c[ImGuiCol_Header]        = ImVec4(0.18f, 0.32f, 0.36f, 0.80f);
    c[ImGuiCol_HeaderHovered] = ImVec4(0.22f, 0.42f, 0.46f, 0.90f);
    c[ImGuiCol_HeaderActive]  = ImVec4(0.24f, 0.48f, 0.52f, 1.00f);
    c[ImGuiCol_WindowBg]      = ImVec4(0.10f, 0.11f, 0.13f, 1.00f);
    style.ScaleAllSizes(scale);

    ImGuiIO &io = ImGui::GetIO();
    const char *fonts[] = {"C:/Windows/Fonts/segoeui.ttf", "C:/Windows/Fonts/arial.ttf"};
    for (const char *f : fonts) {
        if (std::filesystem::exists(f) && io.Fonts->AddFontFromFileTTF(f, 16.f * scale))
            return;
    }
    ImFontConfig cfg;
    cfg.SizePixels = 13.f * scale;
    io.Fonts->AddFontDefault(&cfg);
}

// Reads the requested screen region of the back buffer and writes it as PNG.
void save_capture(ebiv::app::App &app, int fb_w, int fb_h) {
    auto &cap          = app.capture();
    const ImVec2 scale = ImGui::GetIO().DisplayFramebufferScale;
    const int x0 = std::clamp(static_cast<int>(cap.min.x * scale.x), 0, fb_w);
    const int y0 = std::clamp(static_cast<int>(cap.min.y * scale.y), 0, fb_h);
    const int x1 = std::clamp(static_cast<int>(cap.max.x * scale.x), 0, fb_w);
    const int y1 = std::clamp(static_cast<int>(cap.max.y * scale.y), 0, fb_h);
    if (x1 - x0 < 2 || y1 - y0 < 2) {
        app.capture_finished(false, "the Viewer is not visible");
        return;
    }
    const int w = x1 - x0, h = y1 - y0;
    std::vector<uint8_t> pixels(size_t(w) * h * 4);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadBuffer(GL_BACK);
    glReadPixels(x0, fb_h - y1, w, h, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
    cv::Mat rgba(h, w, CV_8UC4, pixels.data()), bgr;
    cv::flip(rgba, rgba, 0);
    cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);
    std::vector<uchar> png;
    if (!cv::imencode(".png", bgr, png)) {
        app.capture_finished(false, "PNG encoding failed");
        return;
    }
    std::ofstream out(cap.path, std::ios::binary | std::ios::trunc);
    out.write(reinterpret_cast<const char *>(png.data()), static_cast<std::streamsize>(png.size()));
    out.close();
    app.capture_finished(static_cast<bool>(out), "cannot write the file");
}

} // namespace

int main(int argc, char **argv) {
    evs::app::launch_console::hide(); // the window of a double-clicked EBIVStudio.bat
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return 1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_MAXIMIZED, GLFW_TRUE);
    GLFWwindow *window = glfwCreateWindow(1600, 950, "EBIV Studio - event-based imaging velocimetry", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    glfwSetDropCallback(window, drop_callback);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable | ImGuiConfigFlags_NavEnableKeyboard;
    io.IniFilename = "EBIVStudio_layout.ini";
    float xscale = 1.f, yscale = 1.f;
    glfwGetWindowContentScale(window, &xscale, &yscale);
    setup_style(std::max(1.f, xscale));
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    {
        ebiv::app::App app;
        // EBIVStudio [parameters.cfg] [recording] [--run] [--console]; parameters are applied before the recording is opened
        std::vector<std::filesystem::path> files;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--console")
                continue; // keep the console of EBIVStudio.bat (launch_console)
            if (arg == "--run")
                app.run_after_load();
            else
                files.push_back(std::filesystem::u8path(arg));
        }
        std::stable_partition(files.begin(), files.end(), [](const std::filesystem::path &p) {
            const auto ext = p.extension().string();
            return ext == ".cfg" || ext == ".ini" || ext == ".CFG";
        });
        for (const auto &f : files)
            app.open_file(f);

        while (!glfwWindowShouldClose(window) && !app.quit_requested()) {
            glfwPollEvents();
            if (glfwGetWindowAttrib(window, GLFW_ICONIFIED)) {
                ImGui_ImplGlfw_Sleep(20);
                continue;
            }
            if (!g_dropped_files.empty()) {
                for (const auto &f : g_dropped_files)
                    app.open_file(f);
                g_dropped_files.clear();
            }
            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();
            app.frame();
            ImGui::Render();
            int w, h;
            glfwGetFramebufferSize(window, &w, &h);
            glViewport(0, 0, w, h);
            glClearColor(0.08f, 0.08f, 0.10f, 1.f);
            glClear(GL_COLOR_BUFFER_BIT);
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
            if (app.capture().pending)
                save_capture(app, w, h);
            glfwSwapBuffers(window);
        }
        app.shutdown();
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    evs::app::launch_console::close();
    return 0;
}
