cmake_minimum_required(VERSION 3.25)
project(kestrel LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(KESTREL_BUILD_TESTS "Build unit tests" ON)

# Flight-software core: portable, float, no heap (enforced from M7)
add_library(gnc_core STATIC fsw/core/src/angles.cpp)
target_include_directories(gnc_core PUBLIC fsw/core/include)
if(MSVC)
  target_compile_options(gnc_core PRIVATE /W4 /permissive-)
else()
  target_compile_options(gnc_core PRIVATE
    -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Wdouble-promotion)
endif()

if(KESTREL_BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  enable_testing()
  add_executable(gnc_core_tests tests/unit/test_angles.cpp)
  target_link_libraries(gnc_core_tests PRIVATE gnc_core GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(gnc_core_tests)
endif()
