cmake_minimum_required(VERSION 3.20)
project(EBIVStudio VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
    add_compile_options(/W3 /permissive- /utf-8 /MP /Zi)
    add_link_options(/DEBUG /OPT:REF /OPT:ICF) # PDBs for Release builds (crash / hang diagnostics)
    add_compile_definitions(NOMINMAX WIN32_LEAN_AND_MEAN _CRT_SECURE_NO_WARNINGS)
endif()

# OpenEB from the build tree (see build.bat): MetavisionSDK_DIR / MetavisionHAL_DIR are passed on the command line.
# Found here (not only in EventStudio) so that the imported targets are visible to every target of this project.
find_package(MetavisionSDK COMPONENTS base core stream REQUIRED)
find_package(MetavisionHAL REQUIRED)
find_package(OpenCV COMPONENTS core imgproc imgcodecs videoio REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(OpenGL REQUIRED)

option(EBIV_BUILD_TESTS "Build the unit tests" ON)

# ---------------------------------------------------------------- EventStudio (git submodule)
# Provides evs_core (filters, hot pixels), evs_io (event file reading), imgui and the platform dialogs.
set(EVENTSTUDIO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/EventStudio)
if(NOT EXISTS ${EVENTSTUDIO_DIR}/CMakeLists.txt)
    message(FATAL_ERROR "external/EventStudio is empty: run  git submodule update --init")
endif()
set(EVS_BUILD_APP OFF CACHE BOOL "" FORCE)
set(EVS_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory(${EVENTSTUDIO_DIR} EventStudio)

# ---------------------------------------------------------------- EBIV: event-based imaging velocimetry
# GUI-free processing library (loading, pre-processing, pseudo images, motion compensation, correlation, export)
add_library(ebiv STATIC
    src/ebiv/events.cpp
    src/ebiv/params.cpp
    src/ebiv/sampling.cpp
    src/ebiv/motion_comp.cpp
    src/ebiv/correlation.cpp
    src/ebiv/processor.cpp
    src/ebiv/postprocess.cpp
    src/ebiv/preprocess.cpp
    src/ebiv/pulse.cpp
    src/ebiv/export.cpp
)
target_include_directories(ebiv PUBLIC src)
target_link_libraries(ebiv PUBLIC evs_io opencv_core)

add_executable(EBIVStudio WIN32
    src/ebiv_app/main.cpp
    src/ebiv_app/app.cpp
    src/ebiv_app/panels.cpp
    src/ebiv_app/viewer.cpp
    src/ebiv_app/render.cpp
    ${EVENTSTUDIO_DIR}/src/app/platform.cpp
)
target_link_libraries(EBIVStudio PRIVATE ebiv imgui opencv_imgproc opencv_imgcodecs comdlg32 shell32 ole32)
if(MSVC)
    # WIN32 executable but keep a regular main()
    target_link_options(EBIVStudio PRIVATE /ENTRY:mainCRTStartup)
endif()

# batch processing with the same library
add_executable(ebiv_cli src/ebiv_app/cli.cpp)
target_link_libraries(ebiv_cli PRIVATE ebiv)

# ---------------------------------------------------------------- tests
if(EBIV_BUILD_TESTS)
    enable_testing()
    find_package(GTest CONFIG REQUIRED)
    add_executable(ebiv_tests tests/ebiv_test.cpp)
    target_link_libraries(ebiv_tests PRIVATE ebiv GTest::gtest GTest::gtest_main)
    include(GoogleTest)
    gtest_discover_tests(ebiv_tests DISCOVERY_TIMEOUT 60 PROPERTIES TIMEOUT 600)
endif()
