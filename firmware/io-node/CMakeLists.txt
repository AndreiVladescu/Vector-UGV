cmake_minimum_required(VERSION 3.16)
project(io_node C)

# Host build: the IO MCU logic and its tests against a simulated radio.
# The STM32 build lives in stm32/ and links the same sources.

set(CMAKE_C_STANDARD 11)
add_compile_options(-Wall -Wextra -Werror=implicit-function-declaration)

add_library(io_core STATIC src/io.c src/link.c src/frames.c src/sx1276.c)
target_include_directories(io_core PUBLIC src)
target_link_libraries(io_core PUBLIC m)

add_library(io_sim STATIC sim/sx1276_sim.c)
target_include_directories(io_sim PUBLIC sim)
target_link_libraries(io_sim PUBLIC io_core)

enable_testing()
add_executable(test_io test/test_io.c)
target_link_libraries(test_io io_sim)
add_test(NAME test_io COMMAND test_io)
