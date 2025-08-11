cmake_minimum_required(VERSION 3.14)
project(the_code LANGUAGES C CXX)

# Set C/C++ standard
set(CMAKE_CXX_STANDARD 20)

# Enable testing framework CTest (should be near the top)
# Required for gtest_discover_tests to work correctly
enable_testing()

# Include directories
include_directories(
        platforms/shared/include
        platforms/falcon-2025/Core/Inc
)

# Add host-tools subdirectory - this is where all host-side tools are defined
add_subdirectory(host-tools)

# Add tests subdirectory (only when not cross-compiling for STM32)
if(NOT CMAKE_TOOLCHAIN_FILE MATCHES ".*stm32_toolchain.cmake")
    message(STATUS "Adding tests subdirectory")
    add_subdirectory(tests)
endif()

# --- STM32 Target Definition ---
if(CMAKE_TOOLCHAIN_FILE MATCHES ".*stm32_toolchain.cmake")
    message(STATUS "Building for STM32 target")

    # Define the STM32 target properties
    set(STM32_TARGET_NAME falcon-2025)
    set(STM32_MCU_FAMILY STM32L4S5xx)
    set(STM32_LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/platforms/falcon-2025/STM32L4S5VITX_FLASH.ld)

    # Check if linker script exists
    if(NOT EXISTS ${STM32_LINKER_SCRIPT})
        message(FATAL_ERROR "STM32 Linker script not found at ${STM32_LINKER_SCRIPT}")
    endif()

    # Include directories (relative to this CMakeLists.txt)
    include_directories(
            platforms/shared/include
            platforms/falcon-2025/Core/Inc
            platforms/falcon-2025/Drivers/STM32L4xx_HAL_Driver/Inc
            platforms/falcon-2025/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy
            platforms/falcon-2025/Drivers/CMSIS/Device/ST/STM32L4xx/Include
            platforms/falcon-2025/Drivers/CMSIS/Include
    )

    # Find source files (adjust patterns if needed)
    file(GLOB_RECURSE STM32_CORE_SOURCES
        "platforms/shared/src/*.cpp"
        "platforms/falcon-2025/Core/Src/*.c"
        "platforms/falcon-2025/Core/Src/*.cpp"
    )
    file(GLOB_RECURSE STM32_DRIVER_SOURCES
        "platforms/falcon-2025/Drivers/STM32L4xx_HAL_Driver/Src/*.c"
        # Add other driver sources if needed, e.g., CMSIS RTOS
    )
    set(STM32_STARTUP_FILE platforms/falcon-2025/Core/Startup/startup_stm32l4s5vitx.s)

    # Tell CMake to treat all found .c files as C++ to mimic IDE behavior
    foreach(C_SOURCE_FILE ${STM32_CORE_SOURCES} ${STM32_DRIVER_SOURCES})
        if(C_SOURCE_FILE MATCHES \.c$)
            set_source_files_properties(${C_SOURCE_FILE} PROPERTIES LANGUAGE CXX)
            #message(STATUS "Treating ${C_SOURCE_FILE} as CXX") # Optional: for verification
        endif()
    endforeach()

    if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${STM32_STARTUP_FILE})
         message(WARNING "STM32 Startup file not found at ${CMAKE_CURRENT_SOURCE_DIR}/${STM32_STARTUP_FILE}")
    endif()

    # Combine final source list: All core .c files + unique .cpp files + drivers + startup
    set(STM32_SOURCES
        ${STM32_CORE_SOURCES}
        ${STM32_DRIVER_SOURCES}
        ${STM32_STARTUP_FILE}
    )

    # Define the executable
    add_executable(${STM32_TARGET_NAME}.elf ${STM32_SOURCES})

    # Target-specific compile definitions
    target_compile_definitions(${STM32_TARGET_NAME}.elf PRIVATE
        $<$<CONFIG:Debug>:DEBUG> # Define DEBUG only for Debug builds
        USE_HAL_DRIVER
        ${STM32_MCU_FAMILY}
    )

    # Linker script and libraries
    # target_link_options(${STM32_TARGET_NAME}.elf PRIVATE -T${STM32_LINKER_SCRIPT}) # Linker script is now handled by toolchain CMAKE_EXE_LINKER_FLAGS_INIT
    target_link_libraries(${STM32_TARGET_NAME}.elf PRIVATE m c)

    # Generate .bin and .hex files (optional)
    add_custom_command(TARGET ${STM32_TARGET_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${STM32_TARGET_NAME}.elf> ${STM32_TARGET_NAME}.bin
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${STM32_TARGET_NAME}.elf> ${STM32_TARGET_NAME}.hex
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${STM32_TARGET_NAME}.elf>
        COMMENT "Generating .bin, .hex, and printing size for ${STM32_TARGET_NAME}"
    )

endif()
