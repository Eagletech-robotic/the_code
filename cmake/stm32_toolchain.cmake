# CMake Toolchain file for STM32 (ARM Cortex-M4)

# Target system configuration
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Specify the cross-compiler
# TODO: Consider making the toolchain path configurable or relying on PATH
set(TOOLCHAIN_PREFIX arm-none-eabi-)
set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}as)
set(CMAKE_AR ${TOOLCHAIN_PREFIX}ar CACHE FILEPATH "Archiver")
set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}objcopy CACHE FILEPATH "Objcopy")
set(CMAKE_OBJDUMP ${TOOLCHAIN_PREFIX}objdump CACHE FILEPATH "Objdump")
set(CMAKE_SIZE ${TOOLCHAIN_PREFIX}size CACHE FILEPATH "Size utility")

# Compiler flags based on STM32CubeIDE output
set(CPU_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb")
set(COMMON_FLAGS "${CPU_FLAGS} -Wall -fdata-sections -ffunction-sections -fstack-usage --specs=nano.specs")

set(CMAKE_C_FLAGS_INIT "${COMMON_FLAGS} -std=gnu11")
set(CMAKE_CXX_FLAGS_INIT "${COMMON_FLAGS} -std=c++17") # Use a C++ standard, not gnu11 for C++
set(CMAKE_ASM_FLAGS_INIT "${CPU_FLAGS}")

# Linker flags based on STM32CubeIDE output
set(LINKER_SCRIPT_FLAG "-T${CMAKE_SOURCE_DIR}/falcon/STM32L475VGTX_FLASH.ld") # Assume build dir is one level down
set(CMAKE_EXE_LINKER_FLAGS_INIT " \
    ${LINKER_SCRIPT_FLAG} \
    ${CPU_FLAGS} \
    --specs=nosys.specs \
    -Wl,-Map=\"${CMAKE_BINARY_DIR}/${PROJECT_NAME}.map\" \
    -Wl,--gc-sections \
    -static \
    -u _printf_float \
    -Wl,--start-group \
    -lc \
    -lm \
    -Wl,--end-group \
")

# Debug/Release flags
set(CMAKE_C_FLAGS_DEBUG_INIT "-g3 -O0")
set(CMAKE_CXX_FLAGS_DEBUG_INIT "-g3 -O0")
set(CMAKE_ASM_FLAGS_DEBUG_INIT "-g3")

set(CMAKE_C_FLAGS_RELEASE_INIT "-Os") # Example Release optimization
set(CMAKE_CXX_FLAGS_RELEASE_INIT "-Os")
set(CMAKE_ASM_FLAGS_RELEASE_INIT "")

# Set find root path (optional, adjust if necessary)
# set(CMAKE_FIND_ROOT_PATH /path/to/your/arm/toolchain)
# set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
# set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
# set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY) 