cmake_minimum_required(VERSION 3.10)
project(host_tools)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)

# Include directories 
# Use a more robust path that works both when built standalone and from the root
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../helloworld2/Core/Inc)

# Compiler flags - match exactly what's in the README command
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O2")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -O2")

# Collect source files exactly as listed in the README command
file(GLOB UTILS_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/../helloworld2/Core/Src/utils/*.cpp"
)

# Only build thibault_debug for native builds (not with Emscripten)
if(NOT EMSCRIPTEN)
    # Thibault debug terminal - match the exact files used in the README command
    # Use pelle.cpp directly instead of pelle.c which just includes pelle.cpp
    add_executable(thibault_debug 
        ${CMAKE_CURRENT_SOURCE_DIR}/thibault_debug.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../helloworld2/Core/Src/eaglesteward/thibault.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../helloworld2/Core/Src/eaglesteward/pelle.cpp
        ${UTILS_SOURCES}
    )
endif()

# Simulator connector (WASM binary) - kept separate since it uses Emscripten
if(EMSCRIPTEN)
    # Required source files for the simulator connector - match the exact order from the manual command
    add_executable(simulator_connector
        ${CMAKE_CURRENT_SOURCE_DIR}/simulator-connector.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../helloworld2/Core/Src/eaglesteward/falcon.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../helloworld2/Core/Src/eaglesteward/pelle.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../helloworld2/Core/Src/eaglesteward/thibault.cpp
        ${UTILS_SOURCES}
    )
    
    # Add specific flags and definitions for the simulator to match manual command exactly
    target_compile_definitions(simulator_connector PRIVATE THIBAULT_AUTOPILOT)
    
    # Set the exact Emscripten flags from the manual command
    set_target_properties(simulator_connector PROPERTIES 
        OUTPUT_NAME "simulator-connector"
        SUFFIX ".wasm"
    )
    
    # Reset all flags
    set(CMAKE_CXX_FLAGS "")
    set(CMAKE_C_FLAGS "")
    set(CMAKE_EXE_LINKER_FLAGS "")
    
    # Set exact flags on the target directly to match the manual command
    target_compile_options(simulator_connector PRIVATE "-o2" "-g" "-s" "ASSERTIONS=1")
    target_link_options(simulator_connector PRIVATE "-o2" "-g" "-s" "ASSERTIONS=1")
    
    # Ensure we're only generating a .wasm file, not .js files
    set(CMAKE_EXECUTABLE_SUFFIX ".wasm")
endif() 