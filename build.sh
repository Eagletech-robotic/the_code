#!/bin/bash
# Build script for robot project following standard build approach

# Exit on error
set -e

# Target directories once for all formatting operations
FORMAT_DIRS="helloworld2/Core/Inc helloworld2/Core/Src"

# Helper function for format pre-checks
_pre_format_checks() {
    if ! command -v clang-format &> /dev/null; then
        echo "Error: clang-format could not be found. Please install it with your package manager." >&2
        echo "Example: sudo apt install clang-format" >&2
        exit 1
    fi
    if [ ! -f .clang-format ]; then
        echo "Error: .clang-format file not found in the project root." >&2
        echo "Create one with: clang-format -style=llvm -dump-config > .clang-format" >&2
        exit 1
    fi
}

# Helper function to run clang-format on specific subdirs
_run_clang_format() {
    echo "Running clang-format on subdirectories of: ${FORMAT_DIRS}"
    find ${FORMAT_DIRS} -mindepth 2 -type f \
         \( -iname '*.c' -o -iname '*.cpp' -o -iname '*.h' -o -iname '*.hpp' \) \
         -exec clang-format -i {} \;
    return $?
}

# Format C/C++ code using clang-format
format_code() {
    echo "=== Formatting code ==="
    _pre_format_checks
    _run_clang_format
    echo "Code formatting complete."
}

# Check C/C++ code format using clang-format
check_format() {
    echo "=== Checking code format ==="
    _pre_format_checks
    
    TEMP_DIR=$(mktemp -d)
    CHANGED=0
    
    FILES_TO_CHECK=$(find ${FORMAT_DIRS} -mindepth 2 -type f \
        \( -iname '*.c' -o -iname '*.cpp' -o -iname '*.h' -o -iname '*.hpp' \))
    
    echo "Checking formatting of $(echo "$FILES_TO_CHECK" | wc -l) files..."
    
    for file in $FILES_TO_CHECK; do
        rel_path=${file#*/}
        backup_dir="$TEMP_DIR/$(dirname "$rel_path")"
        mkdir -p "$backup_dir"
        cp "$file" "$backup_dir/$(basename "$file")"
    done
    
    echo "Temporarily applying clang-format..."
    _run_clang_format
    
    echo "Checking for formatting changes..."
    for file in $FILES_TO_CHECK; do
        rel_path=${file#*/}
        original="$TEMP_DIR/$rel_path"
        
        if ! diff -q "$original" "$file" >/dev/null 2>&1; then
            if [ $CHANGED -eq 0 ]; then
                echo "The following files need formatting:"
                CHANGED=1
            fi
            echo "  - $file"
            diff -u "$original" "$file" | head -n 50
            cp "$original" "$file"
        fi
    done
    
    rm -rf "$TEMP_DIR"
    
    if [ $CHANGED -eq 1 ]; then
        echo "Error: Code formatting check failed." >&2
        echo "To fix: Run './build.sh --format' and commit the changes." >&2
        exit 1
    else
        echo "Code format check passed."
        exit 0
    fi
}

# Parse arguments
TARGET="all"
RUN_FORMAT=false
CHECK_FORMAT=false
TARGET_NATIVE=false
TARGET_WASM=false
TARGET_STM32=false
BUILD_TYPE="Debug"

print_usage() {
    echo "Usage: $0 [OPTIONS]"
    echo "Build options:"
    echo "  --native         Build native tools"
    echo "  --wasm           Build WebAssembly tools"
    echo "  --stm32          Build STM32 firmware"
    echo "  --all            Build native, WebAssembly, and STM32 (default if no target specified)"
    echo "  --clean          Clean build directories before building"
    echo "  --format         Format C/C++ code in Core/Src and Core/Inc using clang-format"
    echo "  --check-format   Check C/C++ code format in Core/Src and Core/Inc; exit with 1 if changes needed"
    echo "Target options:"
    echo "  --debug          Build with debug configuration"
    echo "  --release        Build with release configuration (default)"
    echo "Example:"
    echo "  $0 --wasm --clean    # Clean and build WebAssembly tools"
}

# Check if any arguments were provided
if [ $# -eq 0 ]; then
    print_usage
    exit 0
fi

# Parse command line arguments
for arg in "$@"; do
    case $arg in
        --native)
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
        --format)
            RUN_FORMAT=true
            ;;
        --check-format)
            CHECK_FORMAT=true
            ;;
        --debug)
            BUILD_TYPE="Debug"
            ;;
        --release)
            BUILD_TYPE="Release"
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

# Handle formatting or checking first, as they might exit
if [ "$CHECK_FORMAT" = true ]; then
    check_format
fi

if [ "$RUN_FORMAT" = true ]; then
    format_code
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
    make -j$(nproc)
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
    emmake make -j$(nproc)
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