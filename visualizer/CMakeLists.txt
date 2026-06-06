# Optional SFML-based track visualizer
# Only built if SFML is available on the system.
#
# Install on Debian/Ubuntu: sudo apt install libsfml-dev
# Install on Fedora:       sudo dnf install SFML-devel
# Install on Arch Linux:  sudo pacman -S sfml

find_package(SFML 3 COMPONENTS Graphics Window System QUIET)

if(SFML_FOUND)
    add_executable(track_view track_view.cpp)

    target_compile_features(track_view PRIVATE cxx_std_17)

    target_link_libraries(track_view PRIVATE
        SFML::Graphics
        SFML::Window
        SFML::System
    )

    message(STATUS "track_view visualizer: ENABLED (SFML ${SFML_VERSION} found)")
else()
    message(STATUS "track_view visualizer: DISABLED (SFML 3 not found — install libsfml-dev to enable)")
endif()