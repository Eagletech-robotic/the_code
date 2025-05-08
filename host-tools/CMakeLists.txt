cmake_minimum_required(VERSION 3.14)
project(host_tools)

set(CMAKE_CXX_STANDARD 20)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../falcon/Core/Inc
)

file(GLOB_RECURSE SOURCES_NO_STM32_DEPENDENCY
    "${CMAKE_CURRENT_SOURCE_DIR}/../falcon/Core/Src/eaglesteward/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/../falcon/Core/Src/utils/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/../falcon/Core/Src/robotic/*.cpp"
)


# thibault_debug
if (NOT EMSCRIPTEN)
    add_executable(thibault_debug
        ${CMAKE_CURRENT_SOURCE_DIR}/thibault_debug.cpp
        ${SOURCES_NO_STM32_DEPENDENCY}
    )

    target_compile_definitions(thibault_debug PRIVATE THIBAULT_GUIDANCE MYPRINTF_ALWAYS)

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O2")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -O2")
endif ()

# Simulator connector (WASM binary)
if (EMSCRIPTEN)
    add_executable(simulator_connector
        ${CMAKE_CURRENT_SOURCE_DIR}/simulator-connector.cpp
        ${SOURCES_NO_STM32_DEPENDENCY}
    )

    target_compile_definitions(simulator_connector PRIVATE THIBAULT_GUIDANCE MYPRINTF_ALWAYS)

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
        "-sEXPORTED_FUNCTIONS=['_exported_top_init','_exported_top_step','_create_input','_create_output', '_create_bluetooth']"
    )
endif ()