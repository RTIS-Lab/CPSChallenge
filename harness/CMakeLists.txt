cmake_minimum_required(VERSION 3.14)

# ─── Legacy Harness ────────────────────────────────────────────────
add_executable(fmu_harness harness.cpp)
target_compile_features(fmu_harness PRIVATE cxx_std_17)
target_link_libraries(fmu_harness PRIVATE dl)
add_dependencies(fmu_harness LateralMotionControl)

# ─── Modular Modular Scheduler Harness ─────────────────────────────
add_executable(modular_harness 
    run_experiment.cpp
    fmu_wrapper.cpp
    runner.cpp
    policies/immediate_policy.cpp
)
target_compile_features(modular_harness PRIVATE cxx_std_17)
target_include_directories(modular_harness PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(modular_harness PRIVATE dl)
add_dependencies(modular_harness LateralMotionControl)

# Common Definitions
set(FMU_SO_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../LateralMotionControl/binaries/linux64/LateralMotionControl.so")
set(EXAMPLE_DIR  "${CMAKE_CURRENT_SOURCE_DIR}/../examples/example_v_10")

target_compile_definitions(fmu_harness PRIVATE
    FMU_SO_PATH="${FMU_SO_PATH}"
    EXAMPLE_DIR="${EXAMPLE_DIR}"
)

target_compile_definitions(modular_harness PRIVATE
    FMU_SO_PATH="${FMU_SO_PATH}"
    EXAMPLE_DIR="${EXAMPLE_DIR}"
)