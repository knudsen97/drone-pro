cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
# include(~/libraries/pico/pico-sdk/pico_sdk_init.cmake)
include(../pico-sdk/pico_sdk_init.cmake)

project(drone C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(DRONE ${PROJECT_SOURCE_DIR})

# Initialize the SDK
pico_sdk_init()


# add_compile_options(-Wall
#         -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
#         -Wno-unused-function # we have some for the docs that aren't called
#         -Wno-maybe-uninitialized
#         )

#add subdirectories
add_subdirectory(pwm)
add_subdirectory(bmp280)
add_subdirectory(pid)
# add_subdirectory(mpu)
include_directories(mpu)
include_directories(gpio)


