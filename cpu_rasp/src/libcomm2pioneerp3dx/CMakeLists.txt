cmake_minimum_required(VERSION 3.5)
project(pioneer_p3dx)

# Public Includes
include_directories(./include)

# Library
add_library(pioneer_p3dx STATIC
    ./src/pack.c
    ./src/tty.c
    ./src/pioneer_p3dx.c
    )
install(TARGETS pioneer_p3dx DESTINATION lib)
install(FILES include/pioneer_p3dx.h DESTINATION include)

# Example 1: simple
add_executable(simple ./examples/simple.c)
target_link_libraries(simple pioneer_p3dx)
install(TARGETS simple DESTINATION bin)

# Example 2: teleoperation
add_executable(teleop ./examples/teleop.c)
target_link_libraries(teleop pioneer_p3dx)
install(TARGETS teleop DESTINATION bin)

# Tests
enable_testing()
# set(CMAKE_C_FLAGS "-g -fprofile-arcs -ftest-coverage -O0")
add_executable(test_pack ./tests/ut_pack.c)
target_link_libraries(test_pack pthread gcov)
add_test(NAME test_pack COMMAND ./test_pack)

