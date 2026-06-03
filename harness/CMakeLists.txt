cmake_minimum_required(VERSION 3.14)

add_executable(fmu_harness harness.cpp)

target_compile_features(fmu_harness PRIVATE cxx_std_17)

target_link_libraries(fmu_harness PRIVATE dl)

# Depend on the FMU shared library so it's built first
add_dependencies(fmu_harness LateralMotionControl)

# Embed absolute paths as compile-time defaults so the harness
# finds the .so and example data regardless of working directory.
set(FMU_SO_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../LateralMotionControl/binaries/linux64/LateralMotionControl.so")
set(EXAMPLE_DIR  "${CMAKE_CURRENT_SOURCE_DIR}/../examples/example_v_10")

target_compile_definitions(fmu_harness PRIVATE
    FMU_SO_PATH="${FMU_SO_PATH}"
    EXAMPLE_DIR="${EXAMPLE_DIR}"
)