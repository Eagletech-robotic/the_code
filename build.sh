#!/bin/bash
# Build script for robot project following standard build approach

# Exit on error
set -e

# Parse arguments
TARGET="all"
TARGET_NATIVE=false
TARGET_WASM=false
TARGET_STM32=false
BUILD_TYPE="Debug" # Default to Debug to match IDE log

print_usage() {
    echo "Usage: $0 [OPTIONS]"
    echo "Build options:"
    echo "  --native         Build native tools"
    echo "  --wasm           Build WebAssembly tools"
    echo "  --stm32          Build STM32 firmware"
    echo "  --all            Build native, WebAssembly, and STM32 (default if no target specified)"
    echo "  --clean          Clean build directories before building"
    echo "Target options:"
    echo "  --debug          Build with debug configuration"
    echo "  --release        Build with release configuration (default)"
    echo "Example:"
    echo "  $0 --wasm --clean    # Clean and build WebAssembly tools"
}

# Parse command line arguments
for arg in "$@"; do
    case $arg in
        --native) # Explicitly request native
            TARGET_NATIVE=true
            ;;
        --wasm)
            TARGET_WASM=true
            ;;
        --all)
            TARGET_NATIVE=true
            TARGET_WASM=true
            TARGET_STM32=true
            ;;
        --clean)
            CLEAN=true
            ;;
        --debug)
            BUILD_TYPE="Debug"
            ;;
        --release)
            BUILD_TYPE="Release" # Allow overriding to Release
            ;;
        --stm32)
            TARGET_STM32=true
            ;;
        --help|-h)
            print_usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $arg"
            print_usage
            exit 1
            ;;
    esac
done

# Default to 'all' if no specific target was selected
if [ "$TARGET_NATIVE" = false ] && [ "$TARGET_WASM" = false ] && [ "$TARGET_STM32" = false ]; then
    TARGET_NATIVE=true
    TARGET_WASM=true
    TARGET_STM32=true
fi

# Build native target
build_native() {
    echo "=== Building native tools ==="
    
    if [ "$CLEAN" = true ]; then
        echo "Cleaning build directory..."
        rm -rf build
    fi
    
    mkdir -p build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=${BUILD_TYPE}
    make -j$(nproc) # Build all native targets defined
    cd ..
    
    echo "Native build complete."
}

# Build WASM target
build_wasm() {
    echo "=== Building WebAssembly tools ==="
    
    if [ "$CLEAN" = true ]; then
        echo "Cleaning build-wasm directory..."
        rm -rf build-wasm
    fi
    
    mkdir -p build-wasm
    cd build-wasm
    emcmake cmake .. -DCMAKE_BUILD_TYPE=${BUILD_TYPE}
    emmake make -j$(nproc) # Build all WASM targets defined
    cd ..
    
    echo "WebAssembly build complete."
}

# Build STM32 target
build_stm32() {
    echo "=== Building STM32 firmware (helloworld2.elf) ==="

    if [ ! -f cmake/stm32_toolchain.cmake ]; then
        echo "Error: Toolchain file cmake/stm32_toolchain.cmake not found!" >&2
        exit 1
    fi

    if [ "$CLEAN" = true ]; then
        echo "Cleaning build-stm32 directory..."
        rm -rf build-stm32
    fi

    mkdir -p build-stm32
    cd build-stm32
    cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/stm32_toolchain.cmake -DCMAKE_BUILD_TYPE=${BUILD_TYPE}
    make -j$(nproc) helloworld2.elf
    cd ..

    echo "STM32 build complete. Output: build-stm32/helloworld2.elf"
}

# Build targets based on arguments
if [ "$TARGET_NATIVE" = true ]; then 
    build_native
fi

if [ "$TARGET_WASM" = true ]; then
    build_wasm
fi

if [ "$TARGET_STM32" = true ]; then
    build_stm32
fi

echo "Build process completed successfully!" 