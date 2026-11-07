# EBIV Studio

Desktop application (C++17, Dear ImGui) for **event-based imaging velocimetry**: velocity fields from the event
stream of an event camera (Prophesee, OpenEB 5.2). It is a C++ port and graphical interface of the Python EBIV
processing of C. Willert (DLR), [cewdlr/ebiv](https://github.com/cewdlr/ebiv) (MIT licence): same methods and
parameter files, results compatible with its scripts, but interactive and much faster (a 96 M-event recording loads
in ~1.5 s and one evaluation of 63 x 35 samples takes ~0.25 s on 16 threads).

It comes with `ebiv_cli.exe`, a console tool for batch processing with the same options as `ebiv_run.py`.
Event reading, the event filters and the file dialogs come from [EventStudio](https://github.com/RexAttaque/EventStudio),
included as a git submodule.

## Features

Docked workbench: **Pipeline** (left) → **Viewer** (centre: Pseudo image / Results / Pulse analysis tabs) →
**Inspector** (right) → **Log** (bottom).

1. **Recording**: whole file in memory (8 bytes / event; `.raw` / `.hdf5` / `.dat` through OpenEB, EBIV `.evt`),
   times relative to the first event; save the current events as `.evt` for the Python scripts.
2. **Pre-processing**: hot pixel removal, activity (noise) filter, refractory, trail and polarity filters
   (EventStudio's filters), time / region crop. Revertible.
3. **Pseudo images**: event count, binary, ON - OFF, time surface; despeckle; follow the evaluation window
   (sub-volumes / PIV frames A, B or A+B overlay); timeline of the event rate to navigate; PNG / float TIFF export,
   image series.
4. **Pulsed illumination**: mean event rate over one pulse period and pulse onset (`EBI::MeanPulseHistogram` /
   `DetermineOffsetTime`); snaps pseudo images or PIV frames to the pulses.
5. **Evaluation**, multi-threaded, on a single time step or a time series, optionally in a ROI (drawn on the image):
   - *Motion compensation* (contrast maximization): var, SumSq, SumExp, R1, ISOA, MOA; nearest, bilinear or
     *bilinear (Python-compatible)*, which reproduces `sumWarpedEvents()` (it gives the floor pixel the weight
     fx·fy instead of (1-fx)(1-fy));
   - *Sum of correlation*: NT binary slices per sub-volume, normalized FFT cross-correlation per slice, summed
     (`FFT_CrossCorr2D`);
   - *PIV on pseudo-image pairs*: frames at t ± delay/2 (or locked to the light pulses).
   Gaussian / parabolic 3-point peak fit. Parameters load / save as `ebiv_params.cfg` (extra keys are ignored by
   `ebiv_params.py`).
6. **Post-processing**: normalized median test on vx, vy, |v| (`DetectOutliers`), magnitude / event count / peak
   thresholds, replacement by the mean of valid neighbours (Python uses scipy `griddata`).
7. **Display**: background (pseudo image / scalar map), |V|, Vx, Vy, vorticity, divergence, peak, events; px/ms,
   displacement px or m/s (magnification px/mm); hover for values, click a vector to inspect its sample (objective
   map + warped images, or sub-volume projections + correlation plane).
8. **Export**: `.npz` with the layout of `ebiv_run.py` (`(nt, ny, nx, 6)`, y flipped: `plot_velmap.py` reads it),
   CSV (sensor coordinates, raw and validated velocities, m/s), view as PNG.

**Agreement with the Python code**: on a 96 M-event recording (63 x 35 samples), with the Python-compatible
interpolation, 98.7 % (vx) / 99.7 % (vy) of the motion-compensation vectors agree with `ebiv_run.py` within
0.05 px/ms. The default bilinear interpolation uses the correct weights and therefore differs slightly from Python.

## Requirements

Same as EventStudio:

- **Windows 10 / 11, x64.**
- **Visual Studio 2022** or the *VS 2022 Build Tools* with the "Desktop development with C++" workload.
- **OpenEB 5.2.0** built from source with its dependencies from **vcpkg 2024.11.16** (OpenEB's vcpkg manifest
  provides everything needed: OpenCV 4, GLFW 3, HDF5, Boost, GoogleTest). See *Requirements* in the
  [EventStudio README](https://github.com/RexAttaque/EventStudio#requirements) for the OpenEB build commands.
- **git**, to fetch the EventStudio submodule.
- No camera is needed: EBIV Studio processes recordings.

## Installation

Layout expected by the scripts (any root folder works, `C:\Dev` is the example):

```
C:\Dev\
  openeb\                  OpenEB 5.2 sources, built in openeb\build
  vcpkg-2024.11.16\        vcpkg with OpenEB's dependencies in installed\x64-windows
  openeb_env.bat           copy of openeb_env.example.bat from this repository
  EBIVStudio\              this repository (external\EventStudio is the submodule)
```

1. Build OpenEB 5.2 (see the EventStudio README).
2. Clone with the submodule:
   `git clone --recursive https://github.com/RexAttaque/EBIVStudio.git C:\Dev\EBIVStudio`
   (for an existing clone: `git submodule update --init`; `build.bat` also does it when the folder is empty).
3. Copy `openeb_env.example.bat` to `C:\Dev\openeb_env.bat` (skip if EventStudio is already set up there; edit
   the two paths inside if your folders differ).
4. Run `build.bat`. The first run configures CMake into `build\`; it builds `EBIVStudio.exe`, `ebiv_cli.exe` and
   `ebiv_tests.exe` (EventStudio's libraries are built along, its application is not).
5. Start with `EBIVStudio.bat [recording] [ebiv_params.cfg] [--run]` (`--run` evaluates right after loading).

| Script | What it does |
|---|---|
| `build.bat [target]` | Fetches the submodule if needed, configures (first time) and builds into `build\` (Release, with PDBs) |
| `run_tests.bat [gtest args]` | Runs the unit tests (`build\Release\ebiv_tests.exe`) |
| `EBIVStudio.bat [recording] [cfg] [--run]` | Starts EBIV Studio |

Batch processing (run from a terminal where `openeb_env.bat` was called, so the OpenEB DLLs are found):

```
build\Release\ebiv_cli.exe recording.raw [--cfg ebiv_params.cfg] [--method motion_comp|corr_sum|piv] [--t0 USEC]
                           [--interp nearest|bilinear|bilinear_python] [--threads N]
                           [--out result.npz] [--csv result.csv] [--raw]
```

To update the EventStudio code used by EBIV Studio: `git submodule update --remote external/EventStudio`, rebuild,
and commit the new submodule commit.

## Layout

```
src/ebiv        GUI-free EBIV library: events, parameters, sampling, motion compensation, correlation, processor,
                post-processing, pre-processing, pulsed illumination, export
src/ebiv_app    EBIV Studio (ImGui application) and ebiv_cli
tests           GoogleTest suite (ebiv_tests)
external/EventStudio   submodule: evs_core, evs_io, Dear ImGui, platform dialogs
```

## Credits

The EBIV methods, parameters and file formats are those of the Python EBIV code by Christian Willert (DLR),
[cewdlr/ebiv](https://github.com/cewdlr/ebiv); please cite it (see its `CITATION.cff`) when publishing results.
