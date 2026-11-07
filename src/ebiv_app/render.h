#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <imgui.h>

namespace ebiv::app {

enum class Colormap { Gray, GrayInverted, Jet, Viridis, CoolWarm, Count };
const char *to_string(Colormap colormap);

struct Rgba {
    uint8_t r, g, b, a;
};
Rgba colormap(Colormap cm, float t); // t in [0, 1]
ImU32 colormap_u32(Colormap cm, float t, float alpha = 1.f);

// RGBA8 image of w * h values mapped linearly from [lo, hi]; NaN values get `nan_color`.
void colorize(const float *values, int w, int h, float lo, float hi, Colormap cm, std::vector<uint8_t> &rgba,
              Rgba nan_color = {0, 0, 0, 0});

// Percentile range of the finite values (optionally ignoring zeros), e.g. 0.5 / 99.5.
void robust_range(const std::vector<float> &values, float pct_lo, float pct_hi, bool ignore_zero, float &lo, float &hi);

class Texture {
public:
    Texture() = default;
    Texture(const Texture &) = delete;
    Texture &operator=(const Texture &) = delete;
    ~Texture();

    void upload(const uint8_t *rgba, int w, int h, bool nearest = true);
    ImTextureID id() const { return static_cast<ImTextureID>(tex_); }
    bool valid() const { return tex_ != 0 && w_ > 0; }
    int width() const { return w_; }
    int height() const { return h_; }

private:
    unsigned int tex_ = 0;
    int w_ = 0, h_ = 0;
};

// Sensor / image coordinates <-> screen for a zoomable view.
struct ViewTransform {
    ImVec2 origin{0, 0}; // screen position of image pixel (0, 0)
    float scale = 1.f;   // screen pixels per image pixel
    ImVec2 to_screen(float x, float y) const { return ImVec2(origin.x + x * scale, origin.y + y * scale); }
    ImVec2 to_image(const ImVec2 &s) const { return ImVec2((s.x - origin.x) / scale, (s.y - origin.y) / scale); }
};

// Vertical colorbar with min / max / mid labels.
void draw_colorbar(ImDrawList *dl, ImVec2 pmin, ImVec2 pmax, Colormap cm, float lo, float hi, const char *label);

// "Nice" tick spacing for an axis covering `range` with about `target` ticks.
double nice_step(double range, int target);

std::string format_value(double v);

} // namespace ebiv::app
