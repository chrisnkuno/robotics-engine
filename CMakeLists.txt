cmake_minimum_required(VERSION 3.24)

project(
  robotics_engine
  VERSION 0.1.0
  DESCRIPTION "Layered robotics simulation engine scaffold"
  LANGUAGES CXX)

option(REX_ENABLE_CUDA "Enable the experimental CUDA backend target." OFF)
option(REX_ENABLE_PYTHON "Build Python bindings when pybind11 is available." OFF)
option(REX_ENABLE_TESTS "Build smoke tests." ON)
option(REX_ENABLE_TRACY "Enable Tracy profiling zones when Tracy headers are available." OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)

function(rex_configure_target target)
  target_compile_features(${target} PUBLIC cxx_std_20)
  set_target_properties(${target} PROPERTIES POSITION_INDEPENDENT_CODE ON)
  target_include_directories(
    ${target}
    PUBLIC
      $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)

  if(MSVC)
    target_compile_options(${target} PRIVATE /W4 /permissive-)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  endif()

  if(REX_ENABLE_TRACY)
    target_compile_definitions(${target} PUBLIC REX_ENABLE_TRACY=1)
  endif()
endfunction()

add_library(rex_platform STATIC src/platform/handles.cpp)
rex_configure_target(rex_platform)

add_library(rex_math STATIC src/math/types.cpp)
rex_configure_target(rex_math)
target_link_libraries(rex_math PUBLIC rex_platform)

add_library(rex_geometry STATIC src/geometry/shapes.cpp)
rex_configure_target(rex_geometry)
target_link_libraries(rex_geometry PUBLIC rex_math)

add_library(rex_kinematics STATIC src/kinematics/articulation.cpp)
rex_configure_target(rex_kinematics)
target_link_libraries(rex_kinematics PUBLIC rex_math rex_platform)

add_library(rex_collision STATIC src/collision/contact.cpp)
rex_configure_target(rex_collision)
target_link_libraries(rex_collision PUBLIC rex_geometry rex_math rex_platform)

add_library(rex_solver STATIC src/solver/solver.cpp)
rex_configure_target(rex_solver)
target_link_libraries(rex_solver PUBLIC rex_collision rex_math)

add_library(rex_dynamics STATIC src/dynamics/world.cpp)
rex_configure_target(rex_dynamics)
target_link_libraries(rex_dynamics PUBLIC rex_collision rex_geometry rex_kinematics rex_math rex_platform rex_solver)

add_library(rex_sim STATIC src/sim/engine.cpp)
rex_configure_target(rex_sim)
target_link_libraries(rex_sim PUBLIC rex_dynamics rex_solver)

add_library(rex_viewer STATIC src/viewer/controller.cpp src/viewer/demo.cpp src/viewer/replay.cpp src/viewer/svg_renderer.cpp)
rex_configure_target(rex_viewer)
target_link_libraries(rex_viewer PUBLIC rex_sim)

set(REX_SDL2_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/third_party/sdl2-dev/usr/include")
set(REX_SDL2_CONFIG_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/third_party/sdl2-dev/usr/include/x86_64-linux-gnu")
set(REX_SDL2_SHARED_LIB "/lib/x86_64-linux-gnu/libSDL2-2.0.so.0")

add_library(rex_viewer_window STATIC src/viewer/window_app.cpp)
rex_configure_target(rex_viewer_window)
target_include_directories(
  rex_viewer_window
  PRIVATE
    ${REX_SDL2_INCLUDE_DIR}
    ${REX_SDL2_CONFIG_INCLUDE_DIR})
target_link_libraries(rex_viewer_window PUBLIC rex_viewer PRIVATE ${REX_SDL2_SHARED_LIB})

add_library(rex_engine INTERFACE)
target_link_libraries(
  rex_engine
  INTERFACE
    rex_collision
    rex_dynamics
    rex_geometry
    rex_kinematics
    rex_math
    rex_platform
    rex_sim
    rex_solver
    rex_viewer
    rex_viewer_window)

add_executable(rex_viewer_app apps/rex_viewer_main.cpp)
rex_configure_target(rex_viewer_app)
target_link_libraries(rex_viewer_app PRIVATE rex_engine)

add_executable(rex_perf_app apps/rex_perf_main.cpp)
rex_configure_target(rex_perf_app)
target_link_libraries(rex_perf_app PRIVATE rex_engine)

if(REX_ENABLE_CUDA)
  include(CheckLanguage)
  check_language(CUDA)

  if(CMAKE_CUDA_COMPILER)
    enable_language(CUDA)

    add_library(rex_gpu STATIC src/gpu/backend.cu)
    rex_configure_target(rex_gpu)
    target_link_libraries(rex_gpu PUBLIC rex_collision rex_dynamics)
    target_compile_features(rex_gpu PUBLIC cuda_std_20)
    target_link_libraries(rex_engine INTERFACE rex_gpu)
  else()
    message(WARNING "REX_ENABLE_CUDA=ON but no CUDA compiler was found; skipping rex_gpu.")
  endif()
endif()

if(REX_ENABLE_PYTHON)
  set(PYBIND11_FINDPYTHON ON)
  find_package(Python COMPONENTS Interpreter Development REQUIRED)
  find_package(pybind11 CONFIG QUIET)

  if(pybind11_FOUND)
    pybind11_add_module(rex_py src/python/module.cpp)
    target_link_libraries(rex_py PRIVATE rex_engine)

    if(BUILD_TESTING AND REX_ENABLE_TESTS)
      add_test(
        NAME rex_python_api_test
        COMMAND ${Python_EXECUTABLE} ${PROJECT_SOURCE_DIR}/tests/python_api_test.py $<TARGET_FILE:rex_py>)

      set(REX_PROPERTY_TEST_PYTHON "/home/chris/.codex-tooling/pybind11-venv/bin/python")
      if(EXISTS ${REX_PROPERTY_TEST_PYTHON})
        add_test(
          NAME rex_python_property_test
          COMMAND ${REX_PROPERTY_TEST_PYTHON} ${PROJECT_SOURCE_DIR}/tests/python_property_test.py $<TARGET_FILE:rex_py>)
        add_test(
          NAME rex_python_data_test
          COMMAND
            ${REX_PROPERTY_TEST_PYTHON}
            ${PROJECT_SOURCE_DIR}/tests/python_data_test.py
            $<TARGET_FILE:rex_py>
            ${PROJECT_SOURCE_DIR}/python/rex_data.py)
      endif()
    endif()
  else()
    message(WARNING "REX_ENABLE_PYTHON=ON but pybind11 was not found; skipping rex_py.")
  endif()
endif()

if(BUILD_TESTING AND REX_ENABLE_TESTS)
  add_executable(rex_core_tests tests/core_test.cpp)
  rex_configure_target(rex_core_tests)
  target_link_libraries(rex_core_tests PRIVATE rex_engine)
  add_test(NAME rex_core_tests COMMAND rex_core_tests)

  add_executable(rex_viewer_tests tests/viewer_test.cpp)
  rex_configure_target(rex_viewer_tests)
  target_link_libraries(rex_viewer_tests PRIVATE rex_engine)
  add_test(NAME rex_viewer_tests COMMAND rex_viewer_tests)

  add_test(NAME rex_viewer_window_smoke COMMAND rex_viewer_app --demo-window 2)
  set_tests_properties(rex_viewer_window_smoke PROPERTIES ENVIRONMENT "SDL_VIDEODRIVER=dummy")

  add_test(NAME rex_viewer_live_window_smoke COMMAND rex_viewer_app --live-demo-window 3)
  set_tests_properties(rex_viewer_live_window_smoke PROPERTIES ENVIRONMENT "SDL_VIDEODRIVER=dummy")
endif()
