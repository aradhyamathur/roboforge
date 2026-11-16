# Building the controller with CMake instead, for projects with more files,
# libraries to link (Eigen, a solver, libtorch) or an existing CMake setup:
#
#   cmake -S . -B build/cmake && cmake --build build/cmake
#   python3 -m roboforge run app_cmake.yaml --headless --duration 5
cmake_minimum_required(VERSION 3.16)
project(arm_pid CXX)

# Where the SDK keeps roboforge/controller.hpp.
execute_process(COMMAND python3 -m roboforge cpp include
                OUTPUT_VARIABLE ROBOFORGE_INCLUDE OUTPUT_STRIP_TRAILING_WHITESPACE COMMAND_ERROR_IS_FATAL ANY)

add_library(arm_pid SHARED arm_pid.cpp)
target_include_directories(arm_pid PRIVATE ${ROBOFORGE_INCLUDE})
target_compile_features(arm_pid PRIVATE cxx_std_17)
set_target_properties(arm_pid PROPERTIES PREFIX "lib" CXX_VISIBILITY_PRESET default)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)        # for clangd / VS Code IntelliSense
