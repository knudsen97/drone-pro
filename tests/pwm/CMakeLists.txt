add_executable(pwm
        pwm.c)

target_link_libraries(pwm pico_stdlib hardware_pwm hardware_clocks)

# create map/bin/hex file etc.
pico_add_extra_outputs(pwm)
