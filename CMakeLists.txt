cmake_minimum_required(VERSION 3.14)
project(LateralMotionControl C)

add_library(LateralMotionControl SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/LateralMotionControl/sources/LateralMotionControl.c
)

target_include_directories(LateralMotionControl PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/LateralMotionControl/sources
)

target_link_libraries(LateralMotionControl PRIVATE m)

set_target_properties(LateralMotionControl PROPERTIES
    C_STANDARD 11
    POSITION_INDEPENDENT_CODE ON
    PREFIX ""
    OUTPUT_NAME "LateralMotionControl"
)

install(TARGETS LateralMotionControl
    LIBRARY DESTINATION binaries/linux64
)

add_custom_command(TARGET LateralMotionControl POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/LateralMotionControl/binaries/linux64"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "$<TARGET_FILE:LateralMotionControl>"
        "${CMAKE_CURRENT_SOURCE_DIR}/LateralMotionControl/binaries/linux64/LateralMotionControl.so"
    COMMAND ${CMAKE_COMMAND} -E echo
        "Installed into LateralMotionControl/binaries/linux64/"
)

set(FMU_STAGING_DIR "${CMAKE_CURRENT_SOURCE_DIR}/LateralMotionControl")
set(FMU_OUTPUT_FILE "${CMAKE_CURRENT_SOURCE_DIR}/LateralMotionControl.fmu")

add_custom_target(fmu
    DEPENDS LateralMotionControl
    COMMAND ${CMAKE_COMMAND} -E echo "Packing FMU..."
    COMMAND ${CMAKE_COMMAND} -E rm -f "${FMU_OUTPUT_FILE}"
    COMMAND ${CMAKE_COMMAND} -E chdir "${FMU_STAGING_DIR}"
            ${CMAKE_COMMAND} -E tar cf "${FMU_OUTPUT_FILE}" --format=zip
            modelDescription.xml binaries sources
    COMMAND ${CMAKE_COMMAND} -E echo "Packed ${FMU_OUTPUT_FILE}"
)