#include "ebiv/params.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>

namespace ebiv {

namespace {

std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
    return s;
}

std::string trim(const std::string &s) {
    const auto b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos)
        return {};
    const auto e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

// Minimal configparser: [section] / key = value, '#' and ';' comments, keys case-insensitive.
// Keys of [DEFAULT] are visible in every section (as in Python).
class Ini {
public:
    bool read(const std::filesystem::path &path) {
        std::ifstream in(path);
        if (!in)
            return false;
        std::string line, section = "default";
        while (std::getline(in, line)) {
            line = trim(line);
            if (line.empty() || line[0] == '#' || line[0] == ';')
                continue;
            if (line.front() == '[' && line.back() == ']') {
                section = lower(trim(line.substr(1, line.size() - 2)));
                continue;
            }
            const auto eq = line.find_first_of("=:");
            if (eq == std::string::npos)
                continue;
            std::string value = trim(line.substr(eq + 1));
            // inline comments (configparser keeps them by default, but no EBIV value contains " #")
            for (const char *marker : {" #", "\t#", " ;", "\t;"}) {
                const auto c = value.find(marker);
                if (c != std::string::npos)
                    value = trim(value.substr(0, c));
            }
            values_[section][lower(trim(line.substr(0, eq)))] = value;
        }
        return true;
    }
    bool get(const std::string &section, const std::string &key, std::string &out) const {
        for (const std::string &s : {lower(section), std::string("default")}) {
            const auto sec = values_.find(s);
            if (sec == values_.end())
                continue;
            const auto it = sec->second.find(lower(key));
            if (it != sec->second.end()) {
                out = it->second;
                return true;
            }
        }
        return false;
    }
    template <typename T> void num(const std::string &section, const std::string &key, T &out) const {
        std::string s;
        if (!get(section, key, s))
            return;
        try {
            out = static_cast<T>(std::stod(s));
        } catch (...) {}
    }
    void flag(const std::string &section, const std::string &key, bool &out) const {
        std::string s;
        if (!get(section, key, s))
            return;
        s = lower(s);
        if (s == "1" || s == "yes" || s == "true" || s == "on")
            out = true;
        else if (s == "0" || s == "no" || s == "false" || s == "off")
            out = false;
    }

private:
    std::map<std::string, std::map<std::string, std::string>> values_;
};

} // namespace

const char *to_string(Method method) {
    switch (method) {
    case Method::MotionComp:
        return "motion_comp";
    case Method::CorrSum:
        return "corr_sum";
    case Method::Piv:
        return "piv";
    default:
        return "?";
    }
}

const char *display_name(Method method) {
    switch (method) {
    case Method::MotionComp:
        return "Motion compensation";
    case Method::CorrSum:
        return "Sum of correlation";
    case Method::Piv:
        return "PIV (pseudo-image pairs)";
    default:
        return "?";
    }
}

const char *to_string(Objective objective) {
    switch (objective) {
    case Objective::Variance:
        return "var";
    case Objective::SumSq:
        return "SumSq";
    case Objective::SumExp:
        return "SumExp";
    case Objective::R1:
        return "R1";
    case Objective::Isoa:
        return "isoa";
    case Objective::Moa:
        return "MOA";
    default:
        return "?";
    }
}

bool parse_objective(const std::string &text, Objective &objective) {
    const std::string s = lower(text);
    if (s == "var" || s == "variance")
        objective = Objective::Variance;
    else if (s == "sumsq" || s == "sum_of_squares")
        objective = Objective::SumSq;
    else if (s == "sumexp" || s == "sum_of_exponentials")
        objective = Objective::SumExp;
    else if (s == "r1")
        objective = Objective::R1;
    else if (s == "isoa")
        objective = Objective::Isoa;
    else if (s == "moa")
        objective = Objective::Moa;
    else
        return false;
    return true;
}

const char *to_string(Interpolation interpolation) {
    switch (interpolation) {
    case Interpolation::Nearest:
        return "nearest";
    case Interpolation::Bilinear:
        return "bilinear";
    case Interpolation::BilinearPython:
        return "bilinear_python";
    default:
        return "?";
    }
}

bool parse_interpolation(const std::string &text, Interpolation &interpolation) {
    const std::string s = lower(text);
    if (s == "nearest")
        interpolation = Interpolation::Nearest;
    else if (s == "bilinear")
        interpolation = Interpolation::Bilinear;
    else if (s == "bilinear_python")
        interpolation = Interpolation::BilinearPython;
    else
        return false;
    return true;
}

const char *to_string(PseudoMode mode) {
    switch (mode) {
    case PseudoMode::Count:
        return "Event count";
    case PseudoMode::Binary:
        return "Binary";
    case PseudoMode::Signed:
        return "Polarity (ON - OFF)";
    case PseudoMode::TimeSurface:
        return "Time surface (latest event)";
    default:
        return "?";
    }
}

bool load_cfg(const std::filesystem::path &path, Params &p, std::string *error) {
    Ini ini;
    if (!ini.read(path)) {
        if (error)
            *error = "Cannot read " + path.string();
        return false;
    }
    std::string s;
    ini.flag("DEFAULT", "doSingleTimeStep", p.single_time_step);
    ini.num("DEFAULT", "SampleTimeStart", p.t_start);
    ini.num("DEFAULT", "SampleTimeEnd", p.t_end);
    ini.num("DEFAULT", "SampleTimeStep", p.t_step);
    ini.num("DEFAULT", "SampleDuration", p.t_sample);
    ini.num("DEFAULT", "SampleWidth", p.sample_w);
    ini.num("DEFAULT", "SampleHeight", p.sample_h);
    ini.num("DEFAULT", "SampleStepX", p.step_x);
    ini.num("DEFAULT", "SampleStepY", p.step_y);
    if (ini.get("DEFAULT", "EventPolarity", s))
        parse_polarity(s, p.polarity);
    ini.flag("DEFAULT", "doGaussPeakFit", p.gauss_peak_fit);
    if (ini.get("DEFAULT", "EvalMethod", s)) {
        for (int m = 0; m < static_cast<int>(Method::Count); ++m)
            if (lower(s) == to_string(static_cast<Method>(m)))
                p.method = static_cast<Method>(m);
    }

    ini.num("MOTION_COMP", "ScanRangeVxMin", p.vx_min);
    ini.num("MOTION_COMP", "ScanRangeVxMax", p.vx_max);
    ini.num("MOTION_COMP", "ScanRangeVyMin", p.vy_min);
    ini.num("MOTION_COMP", "ScanRangeVyMax", p.vy_max);
    ini.num("MOTION_COMP", "ScanResolX", p.res_x);
    ini.num("MOTION_COMP", "ScanResolY", p.res_y);
    if (ini.get("MOTION_COMP", "ObjectiveFxn", s))
        parse_objective(s, p.objective);
    if (ini.get("MOTION_COMP", "Interpolation", s))
        parse_interpolation(s, p.interpolation);

    ini.num("CORR_SUM", "TimeOffset", p.t_sep);
    ini.num("CORR_SUM", "NT", p.nt);

    ini.num("PIV", "PIV_Delay", p.piv_delay);
    ini.num("PIV", "PIV_Exposure", p.piv_exposure);
    int image = static_cast<int>(p.piv_image);
    ini.num("PIV", "PIV_ImageMode", image);
    if (image >= 0 && image < static_cast<int>(PseudoMode::Num))
        p.piv_image = static_cast<PseudoMode>(image);
    ini.flag("PIV", "PIV_PulseLock", p.piv_pulse_lock);
    ini.num("PIV", "PulseFrequency", p.pulse_freq_hz);
    ini.num("PIV", "PulseOffset", p.pulse_offset_us);
    ini.num("PIV", "PIV_PulseSeparation", p.piv_pulse_separation);

    ini.flag("EBIV_STUDIO", "RoiEnabled", p.roi_enabled);
    ini.num("EBIV_STUDIO", "RoiX", p.roi.x);
    ini.num("EBIV_STUDIO", "RoiY", p.roi.y);
    ini.num("EBIV_STUDIO", "RoiW", p.roi.w);
    ini.num("EBIV_STUDIO", "RoiH", p.roi.h);
    ini.num("EBIV_STUDIO", "Magnification", p.magnification);
    ini.num("EBIV_STUDIO", "Threads", p.threads);
    return true;
}

bool save_cfg(const std::filesystem::path &path, const Params &p, std::string *error) {
    std::ofstream out(path, std::ios::trunc);
    if (!out) {
        if (error)
            *error = "Cannot create " + path.string();
        return false;
    }
    const auto b = [](bool v) { return v ? "True" : "False"; };
    out << "# EBIV processing parameters (written by EBIV Studio, readable by ebiv_params.py)\n\n"
        << "[DEFAULT]\n"
        << "doSingleTimeStep = " << b(p.single_time_step) << "\n"
        << "# event sampling interval in [usec], relative to the first event\n"
        << "SampleTimeStart  = " << p.t_start << "\n"
        << "SampleTimeEnd    = " << p.t_end << "\n"
        << "# sampling increment in [usec]\n"
        << "SampleTimeStep   = " << p.t_step << "\n"
        << "# duration in [usec] for time-slicing\n"
        << "SampleDuration   = " << p.t_sample << "\n"
        << "# sample area in space\n"
        << "SampleWidth      = " << p.sample_w << "\n"
        << "SampleHeight     = " << p.sample_h << "\n"
        << "# sampling increment in space\n"
        << "SampleStepX      = " << p.step_x << "\n"
        << "SampleStepY      = " << p.step_y << "\n"
        << "EventPolarity    = " << to_string(p.polarity) << "\n"
        << "doGaussPeakFit   = " << b(p.gauss_peak_fit) << "\n"
        << "EvalMethod       = " << to_string(p.method) << "\n\n"
        << "[MOTION_COMP]\n"
        << "# scan range for velocity in [pixel/ms]\n"
        << "ScanRangeVxMin   = " << p.vx_min << "\n"
        << "ScanRangeVxMax   = " << p.vx_max << "\n"
        << "ScanRangeVyMin   = " << p.vy_min << "\n"
        << "ScanRangeVyMax   = " << p.vy_max << "\n"
        << "# scan resolution in [pixel/ms]\n"
        << "ScanResolX       = " << p.res_x << "\n"
        << "ScanResolY       = " << p.res_y << "\n"
        << "# var, SumSq, SumExp, MOA, isoa, R1\n"
        << "ObjectiveFxn     = " << to_string(p.objective) << "\n"
        << "# nearest, bilinear (bilinear_python: weighting of the original Python code)\n"
        << "Interpolation    = " << to_string(p.interpolation) << "\n\n"
        << "[CORR_SUM]\n"
        << "# separation in time for event sampling, in [usec]\n"
        << "TimeOffset       = " << p.t_sep << "\n"
        << "# number of correlation planes in sub-volumes\n"
        << "NT               = " << p.nt << "\n\n"
        << "[PIV]\n"
        << "PIV_Delay        = " << p.piv_delay << "\n"
        << "PIV_Exposure     = " << p.piv_exposure << "\n"
        << "# 0 count, 1 binary, 2 polarity, 3 time surface\n"
        << "PIV_ImageMode    = " << static_cast<int>(p.piv_image) << "\n"
        << "PIV_PulseLock    = " << b(p.piv_pulse_lock) << "\n"
        << "PulseFrequency   = " << p.pulse_freq_hz << "\n"
        << "PulseOffset      = " << p.pulse_offset_us << "\n"
        << "PIV_PulseSeparation = " << p.piv_pulse_separation << "\n\n"
        << "[EBIV_STUDIO]\n"
        << "RoiEnabled       = " << b(p.roi_enabled) << "\n"
        << "RoiX             = " << p.roi.x << "\n"
        << "RoiY             = " << p.roi.y << "\n"
        << "RoiW             = " << p.roi.w << "\n"
        << "RoiH             = " << p.roi.h << "\n"
        << "# image magnification in [pixel/mm], 0 for pixel units\n"
        << "Magnification    = " << p.magnification << "\n"
        << "Threads          = " << p.threads << "\n";
    out.close();
    if (out.fail()) {
        if (error)
            *error = "Error while writing " + path.string();
        return false;
    }
    return true;
}

std::vector<double> velocity_candidates(double vmin, double vmax, double res) {
    std::vector<double> v;
    if (!(res > 0) || vmax < vmin)
        return v;
    // np.arange(vmin, vmax + res, res): ceil((vmax + res - vmin) / res) values
    const double n = std::ceil((vmax + res - vmin) / res - 1e-9);
    for (int i = 0; i < static_cast<int>(n); ++i)
        v.push_back(vmin + i * res);
    return v;
}

std::vector<int64_t> time_steps(const Params &p) {
    std::vector<int64_t> t;
    if (p.single_time_step || p.t_step <= 0 || p.t_end <= p.t_start) {
        t.push_back(p.t_start);
        return t;
    }
    for (int64_t t0 = p.t_start; t0 < p.t_end; t0 += p.t_step)
        t.push_back(t0);
    return t;
}

double displacement_time_us(const Params &p) {
    switch (p.method) {
    case Method::MotionComp:
        return static_cast<double>(p.t_sample);
    case Method::CorrSum:
        return static_cast<double>(p.t_sep);
    default: {
        Interval a, b;
        piv_frames(p, p.t_start, a, b);
        return static_cast<double>(b.t1 - a.t1);
    }
    }
}

void piv_frames(const Params &p, int64_t t0, Interval &a, Interval &b) {
    if (p.piv_pulse_lock && p.pulse_freq_hz > 0) {
        const double period = 1e6 / p.pulse_freq_hz;
        const double k      = std::round((static_cast<double>(t0 - p.pulse_offset_us)) / period);
        const double ta     = p.pulse_offset_us + k * period;
        const int sep       = std::max(1, p.piv_pulse_separation);
        a.t1 = static_cast<int64_t>(std::llround(ta));
        a.t2 = static_cast<int64_t>(std::llround(ta + period));
        b.t1 = static_cast<int64_t>(std::llround(ta + sep * period));
        b.t2 = static_cast<int64_t>(std::llround(ta + (sep + 1) * period));
        return;
    }
    a.t1 = t0 - p.piv_delay / 2;
    a.t2 = a.t1 + p.piv_exposure;
    b.t1 = t0 + p.piv_delay / 2;
    b.t2 = b.t1 + p.piv_exposure;
}

Grid sampling_grid(const Params &p, int width, int height) {
    Grid g;
    int x0 = 0, y0 = 0, x1 = width, y1 = height;
    if (p.roi_enabled && p.roi.w > 0 && p.roi.h > 0) {
        x0 = std::clamp(p.roi.x, 0, width);
        y0 = std::clamp(p.roi.y, 0, height);
        x1 = std::clamp(p.roi.x + p.roi.w, 0, width);
        y1 = std::clamp(p.roi.y + p.roi.h, 0, height);
    }
    g.x0 = x0;
    g.y0 = y0;
    g.dx = std::max(1, p.step_x);
    g.dy = std::max(1, p.step_y);
    if (p.sample_w <= 0 || p.sample_h <= 0)
        return g;
    if (x1 - x0 >= p.sample_w)
        g.nx = (x1 - x0 - p.sample_w) / g.dx + 1;
    if (y1 - y0 >= p.sample_h)
        g.ny = (y1 - y0 - p.sample_h) / g.dy + 1;
    return g;
}

} // namespace ebiv
