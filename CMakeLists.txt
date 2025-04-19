cmake_minimum_required(VERSION 3.10)
project(the_code LANGUAGES C CXX)

# Set C/C++ standard
set(CMAKE_CXX_STANDARD 20)

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wshadow -O3")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wshadow -O3")

# Include directories
include_directories(helloworld2/Core/Inc)

# Add host-tools subdirectory - this is where all host-side tools are defined
add_subdirectory(host-tools)

# --- STM32 Target Definition ---
if(CMAKE_TOOLCHAIN_FILE MATCHES ".*stm32_toolchain.cmake")
    message(STATUS "Building for STM32 target")

    # Define the STM32 target properties
    set(STM32_TARGET_NAME helloworld2)
    set(STM32_MCU_FAMILY STM32L475xx)
    set(STM32_LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/helloworld2/STM32L475VGTX_FLASH.ld)

    # Check if linker script exists
    if(NOT EXISTS ${STM32_LINKER_SCRIPT})
        message(FATAL_ERROR "STM32 Linker script not found at ${STM32_LINKER_SCRIPT}")
    endif()

    # Include directories (relative to this CMakeLists.txt)
    include_directories(
        helloworld2/Core/Inc
        helloworld2/Drivers/STM32L4xx_HAL_Driver/Inc
        helloworld2/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy
        helloworld2/Drivers/CMSIS/Device/ST/STM32L4xx/Include
        helloworld2/Drivers/CMSIS/Include
    )

    # --- Source File Collection ---
    # Find all potentially relevant .c and .cpp files in Core/Src and subdirs
    file(GLOB_RECURSE ALL_CORE_C_SOURCES "helloworld2/Core/Src/*.c")
    file(GLOB_RECURSE ALL_CORE_CPP_SOURCES "helloworld2/Core/Src/*.cpp")

    # List .c files that will be compiled as C++
    set(C_AS_CXX_FILES
        # Note: Use relative paths from CMAKE_CURRENT_SOURCE_DIR or absolute paths
        ${CMAKE_CURRENT_SOURCE_DIR}/helloworld2/Core/Src/main.c
    )
    file(GLOB C_AS_CXX_EAGLESTEWARD RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "helloworld2/Core/Src/eaglesteward/*.c")
    file(GLOB C_AS_CXX_IOT01A RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "helloworld2/Core/Src/iot01A/*.c")
    file(GLOB C_AS_CXX_ROBOTIC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "helloworld2/Core/Src/robotic/*.c")
    file(GLOB C_AS_CXX_UTILS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "helloworld2/Core/Src/utils/*.c")
    list(APPEND C_AS_CXX_FILES ${C_AS_CXX_EAGLESTEWARD} ${C_AS_CXX_IOT01A} ${C_AS_CXX_ROBOTIC} ${C_AS_CXX_UTILS})
    list(REMOVE_DUPLICATES C_AS_CXX_FILES) # Ensure uniqueness

    # Get basenames of .c files compiled as C++
    set(C_AS_CXX_BASENAMES)
    foreach(c_file ${C_AS_CXX_FILES})
        get_filename_component(c_basename ${c_file} NAME_WE)
        list(APPEND C_AS_CXX_BASENAMES ${c_basename})
    endforeach()
    list(REMOVE_DUPLICATES C_AS_CXX_BASENAMES)

    # Filter .cpp files: keep only those whose basename is NOT in C_AS_CXX_BASENAMES
    set(UNIQUE_CORE_CPP_SOURCES)
    foreach(cpp_file ${ALL_CORE_CPP_SOURCES})
        get_filename_component(cpp_basename ${cpp_file} NAME_WE)
        list(FIND C_AS_CXX_BASENAMES ${cpp_basename} index)
        if(index EQUAL -1)
            list(APPEND UNIQUE_CORE_CPP_SOURCES ${cpp_file})
        else()
             message(STATUS "Excluding ${cpp_file} because corresponding .c file exists and is compiled as C++")
        endif()
    endforeach()

    # Find driver and startup files
    file(GLOB_RECURSE STM32_DRIVER_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "helloworld2/Drivers/STM32L4xx_HAL_Driver/Src/*.c")
    set(STM32_STARTUP_FILE helloworld2/Core/Startup/startup_stm32l475vgtx.s)

    if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${STM32_STARTUP_FILE})
         message(WARNING "STM32 Startup file not found at ${CMAKE_CURRENT_SOURCE_DIR}/${STM32_STARTUP_FILE}")
    endif()

    # Combine final source list: All core .c files + unique .cpp files + drivers + startup
    set(STM32_SOURCES
        ${ALL_CORE_C_SOURCES}       # Include all .c files found initially
        ${UNIQUE_CORE_CPP_SOURCES}  # Include only .cpp files without a corresponding .c compiled as C++
        ${STM32_DRIVER_SOURCES}
        ${STM32_STARTUP_FILE}
    )
    list(REMOVE_DUPLICATES STM32_SOURCES) # Prevent duplicates if globs overlap somehow
    # --- End Source File Collection ---


    # Define the executable
    add_executable(${STM32_TARGET_NAME}.elf ${STM32_SOURCES})

    # Force specific C files to compile as C++
    # Ensure paths here are relative to CMAKE_CURRENT_SOURCE_DIR or absolute
    set(C_AS_CXX_FILES_FOR_PROP)
    foreach(f ${C_AS_CXX_FILES})
        if(IS_ABSOLUTE ${f})
            list(APPEND C_AS_CXX_FILES_FOR_PROP ${f})
        else()
            list(APPEND C_AS_CXX_FILES_FOR_PROP ${CMAKE_CURRENT_SOURCE_DIR}/${f})
        endif()
    endforeach()
    set_source_files_properties(${C_AS_CXX_FILES_FOR_PROP} PROPERTIES LANGUAGE CXX)


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
