file(GLOB_RECURSE SOURCES_NO_STM32_DEPENDENCY
        "${PROJECT_SOURCE_DIR}/platforms/shared/src/*.cpp"
        "../Core/Src/eaglesteward/*.cpp"
        "../Core/Src/utils/*.cpp"
        "../Core/Src/robotic/*.cpp"
)

# -----------------
# thibault_debug
# -----------------
if (NOT EMSCRIPTEN)
    add_executable(thibault_debug
            ${CMAKE_CURRENT_SOURCE_DIR}/thibault_debug.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/overrides.cpp
            ${SOURCES_NO_STM32_DEPENDENCY}
    )

    target_include_directories(thibault_debug PRIVATE
            ${PROJECT_SOURCE_DIR}/platforms/shared/include
            ../Core/Inc
    )

    target_compile_definitions(thibault_debug PRIVATE
            MYPRINTF_ALWAYS
    )

    target_compile_options(thibault_debug PRIVATE
            # Common options
            -Wall -Wextra
            # Debug options
            $<$<CONFIG:Debug>:-g -O1>
            $<$<CONFIG:Debug>:-fsanitize=address,undefined>
            $<$<CONFIG:Debug>:-fno-omit-frame-pointer>
            $<$<CONFIG:Debug>:-fno-optimize-sibling-calls>
            # Release options
            $<$<CONFIG:Release>:-O3>
            $<$<CONFIG:Release>:-march=native>
    )
endif ()

# ---------------------
# Simulator connector - WASM binary compiled by Emscripten
# ---------------------
if (EMSCRIPTEN)
    add_executable(simulator_connector
            ${CMAKE_CURRENT_SOURCE_DIR}/simulator-connector.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/overrides.cpp
            ${SOURCES_NO_STM32_DEPENDENCY}
    )

    target_include_directories(simulator_connector PRIVATE
            ${PROJECT_SOURCE_DIR}/platforms/shared/include
            ../Core/Inc
    )

    target_compile_definitions(simulator_connector PRIVATE
            MYPRINTF_ALWAYS
    )

    set_target_properties(simulator_connector PROPERTIES
            LINK_FLAGS ""
            OUTPUT_NAME "simulator-connector"
            SUFFIX ".wasm"
    )

    target_compile_options(simulator_connector PRIVATE
            -g
            -O2
            -DSIMULATOR
    )

    target_link_options(simulator_connector PRIVATE
            -g
            -O2 # Used by Emscripten during linking for WASM optimisations
            -sASSERTIONS=1
            "-sEXPORTED_FUNCTIONS=['_exported_top_init','_exported_top_step','_create_input','_create_output', \
        '_create_bluetooth', '_timer_reset', '_timer_get_us']"
    )
endif ()