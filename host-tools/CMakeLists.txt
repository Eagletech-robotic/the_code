cmake_minimum_required(VERSION 3.14)
project(host_tools)

set(CMAKE_CXX_STANDARD 20)

include_directories(
        ../platforms/shared/include
        ../platforms/falcon-2025/Core/Inc
)

file(GLOB_RECURSE SOURCES_NO_STM32_DEPENDENCY
        "../platforms/shared/src/utils/*.cpp"
        "../platforms/shared/src/robotic/*.cpp"
        "../platforms/falcon-2025/Core/Src/eaglesteward/*.cpp"
        "../platforms/falcon-2025/Core/Src/utils/*.cpp"
        "../platforms/falcon-2025/Core/Src/robotic/*.cpp"
)


# thibault_debug
if (NOT EMSCRIPTEN)
    add_executable(thibault_debug
            ${CMAKE_CURRENT_SOURCE_DIR}/thibault_debug.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/overrides.cpp
            ${SOURCES_NO_STM32_DEPENDENCY}
    )

    target_compile_definitions(thibault_debug PRIVATE MYPRINTF_ALWAYS)

    set(CMAKE_CXX_FLAGS_DEBUG "-g -O1 -fsanitize=address,undefined -fno-omit-frame-pointer -fno-optimize-sibling-calls -Wall -Wextra -Wshadow -Wpedantic")
    set(CMAKE_C_FLAGS_DEBUG "-g -O1 -fsanitize=address,undefined -fno-omit-frame-pointer -fno-optimize-sibling-calls -Wall -Wextra -Wshadow -Wpedantic")

    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -flto -ffast-math -funsafe-math-optimizations  -Wall -Wextra")
    set(CMAKE_C_FLAGS_RELEASE "-O3 -march=native -flto -ffast-math -funsafe-math-optimizations -Wall -Wextra")
endif ()

# Simulator connector (WASM binary)
if (EMSCRIPTEN)
    add_executable(simulator_connector
            ${CMAKE_CURRENT_SOURCE_DIR}/simulator-connector.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/overrides.cpp
            ${SOURCES_NO_STM32_DEPENDENCY}
    )

    target_compile_definitions(simulator_connector PRIVATE MYPRINTF_ALWAYS)

    # Emscripten flags
    set_target_properties(simulator_connector PROPERTIES
            OUTPUT_NAME "simulator-connector"
            SUFFIX ".wasm"
    )

    set(CMAKE_EXE_LINKER_FLAGS "")
    # Ensure we're only generating a .wasm file, not .js files
    set(CMAKE_EXECUTABLE_SUFFIX ".wasm")

    # Comiler flags
    target_compile_options(simulator_connector PRIVATE "-O2" "-g" "-DSIMULATOR")

    # Linker flags
    target_link_options(simulator_connector PRIVATE
            "-O2"
            "-g"
            "-sASSERTIONS=1"
            "-sEXPORTED_FUNCTIONS=['_exported_top_init','_exported_top_step','_create_input','_create_output', \
            '_create_bluetooth', '_timer_reset', '_timer_get_us']"
    )
endif ()