cmake_minimum_required(VERSION 3.14) # GoogleTest requires at least 3.14
project(BluetoothTests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

include_directories(
    ${CMAKE_SOURCE_DIR}/falcon/Core/Inc
)

# The program source file(s) we are testing.
# We compile the .cpp directly into the test executable.
set(PROGRAM_SOURCES
    ${CMAKE_SOURCE_DIR}/falcon/Core/Src/robotic/bluetooth.cpp
    ${CMAKE_SOURCE_DIR}/falcon/Core/Src/robotic/eagle_packet.cpp
    ${CMAKE_SOURCE_DIR}/falcon/Core/Src/eaglesteward/world.cpp
    ${CMAKE_SOURCE_DIR}/falcon/Core/Src/eaglesteward/game_entities.cpp
    # ${CMAKE_SOURCE_DIR}/falcon/Core/Src/utils/myprintf.cpp
)

# Find the test source file(s)
file(GLOB TEST_SOURCES "test_*.cpp")

# Fetch GoogleTest using FetchContent
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.17.0 # Use a specific stable tag
)
FetchContent_MakeAvailable(googletest)

# Enable testing for this directory and subdirectories
enable_testing()

# Define the test executable
add_executable(test_runner
    ${PROGRAM_SOURCES}
    ${TEST_SOURCES}
)

# Link the test executable against GoogleTest libraries
target_link_libraries(test_runner
    PRIVATE GTest::gtest_main # Includes gtest and runs tests automatically
)

# Automatically add tests to CTest
include(GoogleTest)
gtest_discover_tests(test_runner)
