add_executable(mpu
mpu.c)

target_link_libraries(mpu pico_stdlib hardware_pwm hardware_clocks hardware_spi hardware_i2c)
# hardware_i2c
# create map/bin/hex file etc.
pico_add_extra_outputs(mpu)