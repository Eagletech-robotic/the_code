cmake_minimum_required(VERSION 3.14)
project(the_code LANGUAGES C CXX)

# Set C/C++ standard
set(CMAKE_CXX_STANDARD 20)

# Enable testing framework CTest (should be near the top)
# Required for gtest_discover_tests to work correctly
enable_testing()

# Common compiler flags
set(COMMON_FLAGS "-Wall -Wextra -Wshadow -O3 -Wno-unused-parameter -Wno-missing-field-initializers")

# Should only apply to GCC target, not to STM32 and WASM targets
#if (NOT CMAKE_TOOLCHAIN_FILE MATCHES ".*stm32_toolchain.cmake")
#   set(COMMON_FLAGS "${COMMON_FLAGS} -g -fsanitize=address,undefined -fno-omit-frame-pointer")
#endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMMON_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COMMON_FLAGS}")

# GCC-specific compiler flags
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-volatile")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-volatile")
endif()

# Include directories
include_directories(falcon/Core/Inc)

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
    set(STM32_TARGET_NAME falcon)
    set(STM32_MCU_FAMILY STM32L475xx)
    set(STM32_LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/falcon/STM32L475VGTX_FLASH.ld)

    # Check if linker script exists
    if(NOT EXISTS ${STM32_LINKER_SCRIPT})
        message(FATAL_ERROR "STM32 Linker script not found at ${STM32_LINKER_SCRIPT}")
    endif()

    # Include directories (relative to this CMakeLists.txt)
    include_directories(
        falcon/Core/Inc
        falcon/Drivers/STM32L4xx_HAL_Driver/Inc
        falcon/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy
        falcon/Drivers/CMSIS/Device/ST/STM32L4xx/Include
        falcon/Drivers/CMSIS/Include
    )

    # Find source files (adjust patterns if needed)
    file(GLOB_RECURSE STM32_CORE_SOURCES
        "falcon/Core/Src/*.c"
        "falcon/Core/Src/*.cpp"
    )
    file(GLOB_RECURSE STM32_DRIVER_SOURCES
        "falcon/Drivers/STM32L4xx_HAL_Driver/Src/*.c"
        # Add other driver sources if needed, e.g., CMSIS RTOS
    )
    set(STM32_STARTUP_FILE falcon/Core/Startup/startup_stm32l475vgtx.s)

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
