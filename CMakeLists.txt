cmake_minimum_required(VERSION 3.25)
project(kestrel LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(KESTREL_BUILD_TESTS "Build unit tests" ON)
option(KESTREL_WARNINGS_AS_ERRORS "Treat compiler warnings as errors (CI turns this on)" OFF)

# One warning set for everything we write, including tests: the math headers
# are templates, so they are only checked where they are instantiated.
add_library(kestrel_warnings INTERFACE)
if(MSVC)
  target_compile_options(kestrel_warnings INTERFACE /W4 /permissive-)
  if(KESTREL_WARNINGS_AS_ERRORS)
    target_compile_options(kestrel_warnings INTERFACE /WX)
  endif()
else()
  target_compile_options(kestrel_warnings INTERFACE
    -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Wdouble-promotion)
  if(KESTREL_WARNINGS_AS_ERRORS)
    target_compile_options(kestrel_warnings INTERFACE -Werror)
  endif()
endif()

# Flight-software core: portable, float, no heap (enforced from M7)
add_library(gnc_core STATIC fsw/core/src/angles.cpp)
target_include_directories(gnc_core PUBLIC fsw/core/include)
target_link_libraries(gnc_core PRIVATE kestrel_warnings)

# Simulator: the world the flight software flies in (double precision). It may use
# the math headers from fsw/core; fsw/ never includes anything from sim/.
add_library(sim_core STATIC sim/src/rigid_body.cpp sim/src/quadrotor.cpp)
target_include_directories(sim_core PUBLIC sim/include)
target_link_libraries(sim_core PUBLIC gnc_core PRIVATE kestrel_warnings)

add_executable(kestrel_sim sim/harness/main.cpp)
target_link_libraries(kestrel_sim PRIVATE sim_core kestrel_warnings)

if(KESTREL_BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SYSTEM)  # third-party headers: don't report their warnings as ours
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  enable_testing()
  add_executable(gnc_core_tests
    tests/unit/test_angles.cpp
    tests/unit/test_vec_mat.cpp
    tests/unit/test_quat.cpp
    tests/unit/test_euler.cpp
    tests/unit/test_rk4.cpp)
  target_include_directories(gnc_core_tests PRIVATE tests/unit)
  target_link_libraries(gnc_core_tests PRIVATE gnc_core kestrel_warnings GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(gnc_core_tests)

  add_executable(sim_tests tests/sim/test_rigid_body.cpp tests/sim/test_quadrotor.cpp)
  target_include_directories(sim_tests PRIVATE tests/unit)
  target_link_libraries(sim_tests PRIVATE sim_core kestrel_warnings GTest::gtest_main)
  gtest_discover_tests(sim_tests)

  # Smoke tests: every scenario runs to completion and writes its log.
  foreach(scenario hover free_fall yaw_spin pitch_kick)
    add_test(NAME sim_smoke.${scenario}
             COMMAND kestrel_sim ${scenario} ${CMAKE_CURRENT_BINARY_DIR}/${scenario}.csv)
  endforeach()
endif()
