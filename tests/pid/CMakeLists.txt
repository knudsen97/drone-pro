add_executable(pid
    pid.c)

target_link_libraries(pid pico_stdlib hardware_pwm hardware_clocks hardware_i2c hardware_spi)

# create map/bin/hex file etc.
pico_add_extra_outputs(pid)
