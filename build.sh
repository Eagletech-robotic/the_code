#!/bin/bash
# Build script for robot project following standard build approach

# Exit on error
set -e

FORMAT_DIRS="platforms/falcon-2025/Core/Inc platforms/falcon-2025/Core/Src"
BUILD_DIR="build"
BUILD_WASM_DIR="build-wasm"
BUILD_STM32_DIR="build-stm32"
TEST_BUILD_DIR="build" # Tests run in the native build dir

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

format_code() {
    echo "=== Formatting code ==="
    _pre_format_checks
    _run_clang_format
    echo "Code formatting complete."
}

check_format() {
    echo "=== Checking code format ==="
    _pre_format_checks

    TEMP_DIR=$(mktemp -d)
    # Trap ensures temporary directory is cleaned up on script exit or interrupt
    trap 'rm -rf -- "$TEMP_DIR"' EXIT

    CHANGED=0

    FILES_TO_CHECK=$(find ${FORMAT_DIRS} -mindepth 2 -type f \
        \( -iname '*.c' -o -iname '*.cpp' -o -iname '*.h' -o -iname '*.hpp' \))

    if [ -z "$FILES_TO_CHECK" ]; then
        echo "No files found to check in ${FORMAT_DIRS}."
        exit 0
    fi

    echo "Checking formatting of $(echo "$FILES_TO_CHECK" | wc -l) files..."

    echo "Backing up files to temporary location..."
    # Use tar for efficiency if many files
    echo "$FILES_TO_CHECK" | tar -cf - -T - | (cd "$TEMP_DIR" && tar -xf -)

    echo "Temporarily applying clang-format..."
    _run_clang_format # This modifies files in the working directory

    echo "Comparing formatting changes..."
    for file in $FILES_TO_CHECK; do
        # Assumes find gives paths starting from ./ or absolute paths
        original="$TEMP_DIR/$file"

        # Check if original exists, handle potential issues if backup failed
        if [ ! -f "$original" ]; then
            echo "Warning: Could not find backup for $file in $TEMP_DIR. Skipping comparison." >&2
            continue
        fi

        if ! diff -q "$original" "$file" >/dev/null 2>&1; then
            if [ $CHANGED -eq 0 ]; then
                echo "The following files need formatting:"
                CHANGED=1
            fi
            echo "  - $file"
            diff -u "$original" "$file" | head -n 50 # Show diff snippet
            cp "$original" "$file" # Restore original file content
        fi
    done

    # Temp dir cleanup is handled by trap

    if [ $CHANGED -eq 1 ]; then
        echo "Error: Code formatting check failed." >&2
        echo "To fix: Run './build.sh format' and commit the changes." >&2
        exit 1
    else
        echo "Code format check passed."
        exit 0
    fi
}

clean() {
    echo "=== Cleaning build directories ==="
    echo "Removing ${BUILD_DIR}..."
    rm -rf "${BUILD_DIR}"
    echo "Removing ${BUILD_WASM_DIR}..."
    rm -rf "${BUILD_WASM_DIR}"
    echo "Removing ${BUILD_STM32_DIR}..."
    rm -rf "${BUILD_STM32_DIR}"
    echo "Clean complete."
}

print_usage() {
    echo "Usage: $0 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  build            Build targets (default: all)"
    echo "  clean            Remove build directories (${BUILD_DIR}, ${BUILD_WASM_DIR}, ${BUILD_STM32_DIR})"
    echo "  format           Format C/C++ code using clang-format"
    echo "  check-format     Check C/C++ code format; exit with 1 if changes needed"
    echo "  help             Show this help message"
    echo "  test             Build and run native unit tests (using Debug config by default)"
    echo ""
    echo "Build Options (only applicable to 'build' command):"
    echo "  --native         Build native tools"
    echo "  --wasm           Build WebAssembly tools"
    echo "  --stm32          Build STM32 firmware"
    echo "  --all            Build native, WebAssembly, and STM32 (default if no target specified for 'build')"
    echo "  --clean          Clean build directories before building"
    echo "  --debug          Build with Debug configuration"
    echo "  --release        Build with Release configuration (default)"
    echo ""
    echo "Examples:"
    echo "  $0 build --wasm --clean    # Clean wasm build dir, then build WebAssembly tools"
    echo "  $0 format                 # Format C/C++ code"
    echo "  $0 clean                  # Remove all build directories"
    echo "  $0 build --native --debug # Build native tools with Debug configuration"
    echo "  $0 test                  # Build and run native tests (uses Debug)"
    echo "  $0 test --release        # Build and run native tests (uses Release)"
}

build_native() {
    local build_type="$1"
    echo "=== Building native tools (${build_type}) ==="

    mkdir -p "${BUILD_DIR}"
    cd "${BUILD_DIR}"
    cmake .. -DCMAKE_BUILD_TYPE=${build_type}
    make -j$(nproc)
    cd ..

    echo "Native build complete."
}

build_wasm() {
    local build_type="$1"
    echo "=== Building WebAssembly tools (${build_type}) ==="

    mkdir -p "${BUILD_WASM_DIR}"
    cd "${BUILD_WASM_DIR}"
    emcmake cmake .. -DCMAKE_BUILD_TYPE=${build_type}
    emmake make -j$(nproc)
    cd ..

    echo "WebAssembly build complete."
}

build_stm32() {
    local build_type="$1"
    echo "=== Building STM32 firmware (${build_type}) (falcon-2025.elf) ==="

    if [ ! -f cmake/stm32_toolchain.cmake ]; then
        echo "Error: Toolchain file cmake/stm32_toolchain.cmake not found!" >&2
        exit 1
    fi

    mkdir -p "${BUILD_STM32_DIR}"
    cd "${BUILD_STM32_DIR}"
    cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/stm32_toolchain.cmake -DCMAKE_BUILD_TYPE=${build_type}
    make -j$(nproc) falcon-2025.elf
    cd ..

    echo "STM32 build complete. Output: ${BUILD_STM32_DIR}/falcon-2025.elf"
}

test_native() {
    local build_type="${1:-Debug}" # Default to Debug for tests
    local cmake_build_dir="${TEST_BUILD_DIR}"
    echo "=== Building and Running Native Tests (${build_type}) ==="

    # 1. Configure (if necessary) - Ensure tests are enabled
    #    We configure in the standard native build directory.
    mkdir -p "${cmake_build_dir}"
    # Check if CMakeCache exists, if not, run configure
    if [ ! -f "${cmake_build_dir}/CMakeCache.txt" ]; then
        echo "Configuring CMake for native build (including tests)..."
        cmake -S . -B "${cmake_build_dir}" -DCMAKE_BUILD_TYPE=${build_type}
    else
         # If already configured, ensure build type matches or reconfigure?
         # For simplicity, let's just proceed. User can clean if needed.
         echo "Build directory already configured. Skipping CMake configure step."
         echo "Run './build.sh clean && ./build.sh test' for a clean test build."
    fi

    # 2. Build the test runner target
    echo "Building test runner..."
    cmake --build "${cmake_build_dir}" --target test_runner -j$(nproc)

    # 3. Run tests using CTest
    echo "Running tests..."
    cd "${cmake_build_dir}"
    ctest --output-on-failure -C ${build_type} # -C specifies the configuration for multi-config generators
    local test_exit_code=$?
    cd ..

    if [ ${test_exit_code} -ne 0 ]; then
        echo "Error: Native tests failed." >&2
        exit ${test_exit_code}
    else
        echo "Native tests passed successfully."
    fi
}

# Main script logic
if [ $# -eq 0 ]; then
    print_usage
    exit 0
fi

COMMAND=$1
shift # Remove command from argument list

# Handle commands that don't take build options first
case $COMMAND in
    clean)
        clean
        exit 0
        ;;
    format)
        format_code
        exit 0
        ;;
    check-format)
        check_format
        # check_format exits itself based on success/failure
        ;;
    test) # Added test command
        # Handled below after parsing common options like --debug/--release
        ;;
    help|-h|--help)
        print_usage
        exit 0
        ;;
    build)
        # Continue below to parse build options
        ;;
    *)
        echo "Unknown command: $COMMAND" >&2
        print_usage
        exit 1
        ;;
esac

# Parse options for the 'build' command
TARGET_NATIVE=false
TARGET_WASM=false
TARGET_STM32=false
DO_CLEAN=false # Renamed from CLEAN to avoid potential conflicts
BUILD_TYPE="Release" # Default to Release

while [ $# -gt 0 ]; do
    case "$1" in
        --native)
            TARGET_NATIVE=true
            shift
            ;;
        --wasm)
            TARGET_WASM=true
            shift
            ;;
        --stm32)
            TARGET_STM32=true
            shift
            ;;
        --all)
            TARGET_NATIVE=true
            TARGET_WASM=true
            TARGET_STM32=true
            shift
            ;;
        --clean)
            DO_CLEAN=true
            shift
            ;;
        --debug)
            BUILD_TYPE="Debug"
            shift
            ;;
        --release)
            BUILD_TYPE="Release"
            shift
            ;;
        *)
            echo "Unknown option for build command: $1" >&2
            print_usage
            exit 1
            ;;
    esac
done

# Handle the 'test' command here after parsing build type
if [ "$COMMAND" = "test" ]; then
    # Currently, tests only run natively.
    # Clean if requested (only cleans the native build dir for tests)
    if [ "$DO_CLEAN" = true ]; then
        echo "Cleaning ${TEST_BUILD_DIR} before testing..."
        rm -rf "${TEST_BUILD_DIR}"
    fi
    test_native "$BUILD_TYPE"
    exit 0 # Exit after running tests
fi

# Default to 'all' targets if 'build' command is given but no specific target flags
if [ "$TARGET_NATIVE" = false ] && [ "$TARGET_WASM" = false ] && [ "$TARGET_STM32" = false ]; then
    echo "No specific target specified for build, defaulting to --all"
    TARGET_NATIVE=true
    TARGET_WASM=true
    TARGET_STM32=true
fi

# Perform clean step if requested for the build command
if [ "$DO_CLEAN" = true ]; then
    echo "Cleaning requested before build..."
    # Selectively clean only the directories for the targets being built
    if [ "$TARGET_NATIVE" = true ]; then
        echo "Cleaning ${BUILD_DIR}..."
        rm -rf "${BUILD_DIR}"
    fi
     if [ "$TARGET_WASM" = true ]; then
        echo "Cleaning ${BUILD_WASM_DIR}..."
        rm -rf "${BUILD_WASM_DIR}"
    fi
     if [ "$TARGET_STM32" = true ]; then
        echo "Cleaning ${BUILD_STM32_DIR}..."
        rm -rf "${BUILD_STM32_DIR}"
    fi
fi


# Execute build functions for selected targets
if [ "$TARGET_NATIVE" = true ]; then
    build_native "$BUILD_TYPE"
fi

if [ "$TARGET_WASM" = true ]; then
    build_wasm "$BUILD_TYPE"
fi

if [ "$TARGET_STM32" = true ]; then
    build_stm32 "$BUILD_TYPE"
fi

echo "Build process completed successfully!"

exit 0