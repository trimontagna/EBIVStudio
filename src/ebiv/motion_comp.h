#pragma once

#include <vector>

#include "ebiv/params.h"
#include "ebiv/sampling.h"

namespace ebiv {

// Motion compensation (contrast maximization), port of ebiv_utils.CalcObjectiveForShift / sumWarpedEvents /
// FindPeakInObjectiveMap.

// Image of warped events (IWE): every event is moved to the centre of the time window, (t - dt/2) * v,
// and accumulated on the w x h grid. Velocities in [px/us]. Returns the number of events used.
int warp_events(const std::vector<SampleEvent> &events, double dt, double vx, double vy, Interpolation interpolation,
                int w, int h, std::vector<float> &iwe);

double objective_value(const std::vector<float> &iwe, Objective objective);

struct ObjectiveMap {
    int nx = 0, ny = 0;            // number of vx / vy candidates
    std::vector<double> value;     // ny * nx, row = vy candidate
    std::vector<int> events;       // events used per candidate
};

// Objective for every (vx, vy) candidate [px/ms]. `iwe` is a scratch buffer.
void objective_map(const std::vector<SampleEvent> &events, double dt, const std::vector<double> &vx,
                   const std::vector<double> &vy, Interpolation interpolation, Objective objective, int w, int h,
                   ObjectiveMap &map, std::vector<float> &iwe);

// 3-point parabolic peak fit (Gaussian when exp_fit and all values > 0), as PeakFit3pt().
double peak_fit_3pt(double x0, double a, double b, double c, double dx, bool exp_fit);

struct ObjectivePeak {
    double vx = 0, vy = 0; // [px/ms]
    double value = 0;      // objective at the peak
    int events   = 0;      // events used at the peak
    int ix = -1, iy = -1;  // candidate indices of the peak, -1 when the map is flat
};
ObjectivePeak find_objective_peak(const ObjectiveMap &map, const std::vector<double> &vx, const std::vector<double> &vy,
                                  bool exp_fit);

} // namespace ebiv
