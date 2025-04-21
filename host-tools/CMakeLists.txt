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
        ${CMAKE_CURRENT_SOURCE_DIR}/../helloworld2/Core/Src/eaglesteward/guidance/thibault.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../helloworld2/Core/Src/eaglesteward/pelle.cpp
        ${UTILS_SOURCES}
    )

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O2")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -O2")
endif()

# Simulator connector (WASM binary)
if(EMSCRIPTEN)
    add_executable(simulator_connector
        ${CMAKE_CURRENT_SOURCE_DIR}/simulator-connector.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../helloworld2/Core/Src/eaglesteward/top_driver.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../helloworld2/Core/Src/eaglesteward/guidance/thibault.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../helloworld2/Core/Src/eaglesteward/pelle.cpp
        ${UTILS_SOURCES}
    )

    target_compile_definitions(simulator_connector PRIVATE THIBAULT_GUIDANCE)

    # Emscripten flags
    set_target_properties(simulator_connector PROPERTIES
        OUTPUT_NAME "simulator-connector"
        SUFFIX ".wasm"
    )

    set(CMAKE_EXE_LINKER_FLAGS "")
    # Ensure we're only generating a .wasm file, not .js files
    set(CMAKE_EXECUTABLE_SUFFIX ".wasm")

    # Comiler flags
    target_compile_options(simulator_connector PRIVATE "-O2" "-g")

    # Linker flags
    target_link_options(simulator_connector PRIVATE 
        "-O2" 
        "-g" 
        "-sASSERTIONS=1" 
        "-sEXPORTED_FUNCTIONS=['_exported_top_init','_exported_top_step','_create_input','_create_output']"
    )
endif()