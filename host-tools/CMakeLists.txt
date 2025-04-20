cmake_minimum_required(VERSION 3.10)
project(host_tools)

set(CMAKE_CXX_STANDARD 20)

# Use a path that works both when built standalone and from the root
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../helloworld2/Core/Inc)

file(GLOB UTILS_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/../helloworld2/Core/Src/utils/*.cpp"
)

# thibault_debug
if(NOT EMSCRIPTEN)
    add_executable(thibault_debug
        ${CMAKE_CURRENT_SOURCE_DIR}/thibault_debug.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../helloworld2/Core/Src/eaglesteward/thibault.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../helloworld2/Core/Src/eaglesteward/pelle.c
        ${UTILS_SOURCES}
    )

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O2")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -O2")
endif()

# Simulator connector (WASM binary)
if(EMSCRIPTEN)
    add_executable(simulator_connector
        ${CMAKE_CURRENT_SOURCE_DIR}/simulator-connector.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../helloworld2/Core/Src/eaglesteward/falcon.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../helloworld2/Core/Src/eaglesteward/thibault.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../helloworld2/Core/Src/eaglesteward/pelle.c
        ${UTILS_SOURCES}
    )

    target_compile_definitions(simulator_connector PRIVATE THIBAULT_AUTOPILOT)

    # Emscripten flags
    set_target_properties(simulator_connector PROPERTIES
        OUTPUT_NAME "simulator-connector"
        SUFFIX ".wasm"
    )

    set(CMAKE_EXE_LINKER_FLAGS "")
    # Ensure we're only generating a .wasm file, not .js files
    set(CMAKE_EXECUTABLE_SUFFIX ".wasm")

    # Set flags on the target directly
    target_compile_options(simulator_connector PRIVATE "-o2" "-g" "-s" "ASSERTIONS=1")
    target_link_options(simulator_connector PRIVATE "-o2" "-g" "-s" "ASSERTIONS=1")
endif()