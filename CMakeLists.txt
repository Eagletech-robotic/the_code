cmake_minimum_required(VERSION 3.30)
project(the_code LANGUAGES C CXX)

# Set C/C++ standard
set(CMAKE_CXX_STANDARD 20)

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wshadow -O3")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wshadow -O3")

# Include directories
include_directories(helloworld2/Core/Inc)

# Collect C++ source files
file(GLOB CPP_SOURCES
        "helloworld2/Core/Src/eaglesteward/thibault.cpp"
        "helloworld2/Core/Src/utils/*.cpp"
)

# Collect C source files
file(GLOB C_SOURCES "helloworld2/Core/Src/utils/*.c")

# Create executable from both C and C++ sources
add_executable(thibault ${CPP_SOURCES} ${C_SOURCES})

# Definitions
target_compile_definitions(thibault PRIVATE STANDALONE)