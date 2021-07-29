add_executable(bmp280
    bmp280.c)

target_link_libraries(bmp280 pico_stdlib hardware_spi)

pico_add_extra_outputs(bmp280)