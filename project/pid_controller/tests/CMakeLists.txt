# Unit tests for the PID class, the error computations and the gains file.
# This build is separate from the main
# pid_controller build so it needs neither CARLA nor the planner sources.
#
# From project/pid_controller:
#   cmake -S tests -B build-tests
#   cmake --build build-tests
#   ctest --test-dir build-tests --output-on-failure

cmake_minimum_required(VERSION 3.10)
project(PidControllerTests CXX)

# Recent Google Test releases require C++17. pid_controller.cpp itself stays
# valid C++14, which is what the main build uses.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(GTest REQUIRED)
find_package(Threads REQUIRED)

add_executable(pid_controller_test
  pid_controller_test.cpp
  control_errors_test.cpp
  pid_gains_test.cpp
  ../pid_controller.cpp)

target_compile_options(pid_controller_test PRIVATE -Wall -Wextra)
target_link_libraries(pid_controller_test GTest::GTest GTest::Main Threads::Threads)

enable_testing()
add_test(NAME pid_controller_test COMMAND pid_controller_test)
