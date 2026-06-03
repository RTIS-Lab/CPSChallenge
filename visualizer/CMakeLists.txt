# Optional SFML-based track visualizer
# Only built if SFML is available on the system.
#
# Install on Debian/Ubuntu: sudo apt install libsfml-dev
# Install on Fedora:       sudo dnf install SFML-devel

find_package(SFML 2 COMPONENTS graphics window system QUIET)

if(SFML_FOUND)
    add_executable(track_view track_view.cpp)

    target_compile_features(track_view PRIVATE cxx_std_17)

    target_link_libraries(track_view PRIVATE
        sfml-graphics
        sfml-window
        sfml-system
    )

    message(STATUS "track_view visualizer: ENABLED (SFML ${SFML_VERSION} found)")
else()
    message(STATUS "track_view visualizer: DISABLED (SFML not found — install libsfml-dev to enable)")
endif()