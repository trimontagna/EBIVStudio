# Third-party notices

## EBIV (event-based imaging velocimetry), C. Willert

EBIV Studio is a C++ re-implementation of the processing of [cewdlr/ebiv](https://github.com/cewdlr/ebiv)
(Python scripts and the C++ `EBI` library of `pyebiv`), ported from commit `a76ef5b`. Its methods, parameter file
format (`ebiv_params.cfg`) and output layout (`.npz`) follow that code, which is distributed under the MIT licence:

```
MIT License

Copyright (c) 2022 Chris Willert (cewdlr)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## Other components

- **EventStudio** (git submodule `external/EventStudio`): event file reading, event filters, platform dialogs.
- **Dear ImGui** v1.91.5-docking (MIT licence, Omar Cornut), vendored in EventStudio under
  `third_party/imgui` with its `LICENSE.txt`.
- Linked at build time, not distributed here: OpenEB 5.2 (Prophesee, Apache 2.0), OpenCV 4, GLFW 3,
  GoogleTest (through vcpkg).
