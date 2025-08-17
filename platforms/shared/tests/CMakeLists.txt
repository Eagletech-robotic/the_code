# The program source file(s) we are testing.
# We compile the .cpp directly into the test executable.
file(GLOB_RECURSE PROGRAM_SOURCES
        "${PROJECT_SOURCE_DIR}/platforms/shared/src/*.cpp"
        ${PROJECT_SOURCE_DIR}/platforms/falcon-2025/Core/Src/eaglesteward/potential_field.cpp
        ${PROJECT_SOURCE_DIR}/platforms/falcon-2025/Core/Src/eaglesteward/game_entities.cpp
        ${PROJECT_SOURCE_DIR}/platforms/falcon-2025/Core/Src/eaglesteward/world.cpp
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

add_executable(shared_tests
        ${PROGRAM_SOURCES}
        ${TEST_SOURCES}
)

target_include_directories(shared_tests PRIVATE
        ${PROJECT_SOURCE_DIR}/platforms/shared/include
        ${PROJECT_SOURCE_DIR}/platforms/falcon-2025/Core/Inc
)

target_link_libraries(shared_tests PRIVATE
        GTest::gtest_main # Includes gtest and runs tests automatically
)

# Automatically add tests to CTest
include(GoogleTest)
gtest_discover_tests(shared_tests)
