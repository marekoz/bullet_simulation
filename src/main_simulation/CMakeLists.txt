cmake_minimum_required(VERSION 3.24)
project(VRGroupSimulation VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(${PROJECT_SOURCE_DIR}/include)

# Fetch EnTT 
if(NOT EXISTS "${PROJECT_SOURCE_DIR}/include/entt")
    message(STATUS "Downloading EnTT...")
    file(DOWNLOAD
        "https://github.com/skypjack/entt/archive/refs/tags/v3.10.0.tar.gz"
        "${PROJECT_BINARY_DIR}/entt.tar.gz"
        SHOW_PROGRESS
    )
    execute_process(COMMAND ${CMAKE_COMMAND} -E tar xzf ${PROJECT_BINARY_DIR}/entt.tar.gz)
    execute_process(COMMAND ${CMAKE_COMMAND} -E rename "entt-3.10.0/src/entt" "${PROJECT_SOURCE_DIR}/include/entt")
    file(REMOVE_RECURSE "${PROJECT_BINARY_DIR}/entt.tar.gz" "${PROJECT_BINARY_DIR}/entt-3.10.0")
endif()

# Fetch Catch2 v2 for testing
if(NOT EXISTS "${PROJECT_SOURCE_DIR}/include/catch2")
    message(STATUS "Downloading Catch2 v2...")
    file(DOWNLOAD
        "https://github.com/catchorg/Catch2/archive/refs/tags/v2.13.10.tar.gz"
        "${PROJECT_BINARY_DIR}/catch2.tar.gz"
        SHOW_PROGRESS
    )
    execute_process(COMMAND ${CMAKE_COMMAND} -E tar xzf ${PROJECT_BINARY_DIR}/catch2.tar.gz)
    execute_process(COMMAND ${CMAKE_COMMAND} -E rename "Catch2-2.13.10/single_include/catch2" "${PROJECT_SOURCE_DIR}/include/catch2")
    file(REMOVE_RECURSE "${PROJECT_BINARY_DIR}/catch2.tar.gz" "${PROJECT_BINARY_DIR}/Catch2-2.13.10")
endif()

# Source files
set(SOURCES
    src/main.cpp
    src/simulation.cpp
)

set(TEST_SOURCES
    tests/tests.cpp
)

# Simulation library
add_library(simulation_lib STATIC src/simulation.cpp)
target_include_directories(simulation_lib PRIVATE include)

# Main executable
add_executable(main src/main.cpp)
target_link_libraries(main PRIVATE simulation_lib)
target_include_directories(main PRIVATE include)

# Test executable
add_executable(tests ${TEST_SOURCES})
target_compile_definitions(tests PRIVATE CATCH_CONFIG_MAIN)
target_include_directories(tests PRIVATE include)
target_link_libraries(tests PRIVATE simulation_lib)
