#include "ebiv_app/render.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include <GLFW/glfw3.h>

#ifndef GL_CLAMP_TO_EDGE
#define GL_CLAMP_TO_EDGE 0x812F
#endif

namespace ebiv::app {

const char *to_string(Colormap colormap) {
    switch (colormap) {
    case Colormap::Gray:
        return "Gray";
    case Colormap::GrayInverted:
        return "Gray (inverted)";
    case Colormap::Jet:
        return "Jet";
    case Colormap::Viridis:
        return "Viridis";
    case Colormap::CoolWarm:
        return "Cool-warm (diverging)";
    default:
        return "?";
    }
}

namespace {

uint8_t u8(float v) { return static_cast<uint8_t>(std::clamp(v, 0.f, 1.f) * 255.f + 0.5f); }

Rgba lerp_table(const float (*table)[3], int n, float t) {
    const float x = std::clamp(t, 0.f, 1.f) * (n - 1);
    const int i   = std::min(n - 2, static_cast<int>(x));
    const float f = x - i;
    return {u8(table[i][0] + (table[i + 1][0] - table[i][0]) * f), u8(table[i][1] + (table[i + 1][1] - table[i][1]) * f),
            u8(table[i][2] + (table[i + 1][2] - table[i][2]) * f), 255};
}

constexpr float kViridis[][3] = {
    {0.267f, 0.005f, 0.329f}, {0.283f, 0.141f, 0.458f}, {0.254f, 0.265f, 0.530f}, {0.207f, 0.372f, 0.553f},
    {0.164f, 0.471f, 0.558f}, {0.128f, 0.567f, 0.551f}, {0.135f, 0.659f, 0.518f}, {0.267f, 0.749f, 0.441f},
    {0.478f, 0.821f, 0.318f}, {0.741f, 0.873f, 0.150f}, {0.993f, 0.906f, 0.144f}};
constexpr float kCoolWarm[][3] = {{0.230f, 0.299f, 0.754f}, {0.552f, 0.690f, 0.996f}, {0.866f, 0.866f, 0.866f},
                                  {0.957f, 0.604f, 0.482f}, {0.706f, 0.016f, 0.150f}};

} // namespace

Rgba colormap(Colormap cm, float t) {
    t = std::isnan(t) ? 0.f : std::clamp(t, 0.f, 1.f);
    switch (cm) {
    case Colormap::Gray:
        return {u8(t), u8(t), u8(t), 255};
    case Colormap::GrayInverted:
        return {u8(1 - t), u8(1 - t), u8(1 - t), 255};
    case Colormap::Jet: {
        const float r = std::clamp(1.5f - std::abs(4.f * t - 3.f), 0.f, 1.f);
        const float g = std::clamp(1.5f - std::abs(4.f * t - 2.f), 0.f, 1.f);
        const float b = std::clamp(1.5f - std::abs(4.f * t - 1.f), 0.f, 1.f);
        return {u8(r), u8(g), u8(b), 255};
    }
    case Colormap::Viridis:
        return lerp_table(kViridis, static_cast<int>(std::size(kViridis)), t);
    case Colormap::CoolWarm:
        return lerp_table(kCoolWarm, static_cast<int>(std::size(kCoolWarm)), t);
    default:
        return {0, 0, 0, 255};
    }
}

ImU32 colormap_u32(Colormap cm, float t, float alpha) {
    const Rgba c = colormap(cm, t);
    return IM_COL32(c.r, c.g, c.b, static_cast<int>(std::clamp(alpha, 0.f, 1.f) * 255));
}

void colorize(const float *values, int w, int h, float lo, float hi, Colormap cm, std::vector<uint8_t> &rgba,
              Rgba nan_color) {
    rgba.resize(size_t(w) * h * 4);
    const float inv = hi > lo ? 1.f / (hi - lo) : 1.f;
    // 256-entry lookup: colorizing a 1 Mpx image per frame stays cheap
    Rgba lut[256];
    for (int i = 0; i < 256; ++i)
        lut[i] = colormap(cm, i / 255.f);
    for (size_t i = 0, n = size_t(w) * h; i < n; ++i) {
        const float v = values[i];
        const Rgba c  = std::isnan(v) ? nan_color : lut[static_cast<int>(std::clamp((v - lo) * inv, 0.f, 1.f) * 255.f)];
        rgba[i * 4 + 0] = c.r;
        rgba[i * 4 + 1] = c.g;
        rgba[i * 4 + 2] = c.b;
        rgba[i * 4 + 3] = c.a;
    }
}

void robust_range(const std::vector<float> &values, float pct_lo, float pct_hi, bool ignore_zero, float &lo, float &hi) {
    std::vector<float> v;
    v.reserve(values.size());
    for (float x : values)
        if (std::isfinite(x) && !(ignore_zero && x == 0.f))
            v.push_back(x);
    if (v.empty()) {
        lo = 0;
        hi = 1;
        return;
    }
    const auto pick = [&](float pct) {
        const size_t k = std::min(v.size() - 1, static_cast<size_t>(pct / 100.f * (v.size() - 1) + 0.5f));
        std::nth_element(v.begin(), v.begin() + k, v.end());
        return v[k];
    };
    lo = pick(pct_lo);
    hi = pick(pct_hi);
    if (!(hi > lo))
        hi = lo + (lo != 0 ? std::abs(lo) * 0.01f : 1.f);
}

Texture::~Texture() {
    if (tex_)
        glDeleteTextures(1, &tex_);
}

void Texture::upload(const uint8_t *rgba, int w, int h, bool nearest) {
    if (w <= 0 || h <= 0)
        return;
    if (!tex_)
        glGenTextures(1, &tex_);
    glBindTexture(GL_TEXTURE_2D, tex_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, nearest ? GL_NEAREST : GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    if (w != w_ || h != h_)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, rgba);
    else
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, rgba);
    w_ = w;
    h_ = h;
}

std::string format_value(double v) {
    char buf[32];
    const double a = std::abs(v);
    if (v == 0)
        return "0";
    if (a >= 1e5 || a < 1e-3)
        std::snprintf(buf, sizeof(buf), "%.2e", v);
    else if (a >= 100)
        std::snprintf(buf, sizeof(buf), "%.0f", v);
    else if (a >= 10)
        std::snprintf(buf, sizeof(buf), "%.1f", v);
    else
        std::snprintf(buf, sizeof(buf), "%.3g", v);
    return buf;
}

double nice_step(double range, int target) {
    if (!(range > 0) || target < 1)
        return 1;
    const double raw = range / target;
    const double mag = std::pow(10.0, std::floor(std::log10(raw)));
    const double f   = raw / mag;
    return (f < 1.5 ? 1 : f < 3.5 ? 2 : f < 7.5 ? 5 : 10) * mag;
}

void draw_colorbar(ImDrawList *dl, ImVec2 pmin, ImVec2 pmax, Colormap cm, float lo, float hi, const char *label) {
    const int n = 64;
    const float h = (pmax.y - pmin.y) / n;
    for (int i = 0; i < n; ++i) {
        const float t = 1.f - (i + 0.5f) / n;
        dl->AddRectFilled(ImVec2(pmin.x, pmin.y + i * h), ImVec2(pmax.x, pmin.y + (i + 1) * h + 0.5f),
                          colormap_u32(cm, t));
    }
    dl->AddRect(pmin, pmax, IM_COL32(200, 200, 200, 200));
    const ImU32 text = IM_COL32(230, 230, 230, 255);
    const float fh   = ImGui::GetFontSize();
    const float x    = pmax.x + 4;
    dl->AddText(ImVec2(x, pmin.y - 2), text, format_value(hi).c_str());
    dl->AddText(ImVec2(x, (pmin.y + pmax.y) * 0.5f - fh * 0.5f), text, format_value(0.5 * (lo + hi)).c_str());
    dl->AddText(ImVec2(x, pmax.y - fh + 2), text, format_value(lo).c_str());
    if (label && *label) {
        const ImVec2 sz = ImGui::CalcTextSize(label);
        dl->AddText(ImVec2(std::max(pmin.x, pmax.x - sz.x), pmin.y - fh - 4), text, label);
    }
}

} // namespace ebiv::app
