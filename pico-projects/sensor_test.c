#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h" 
#include "pico/binary_info.h" 
#include "hardware/i2c.h"

const uint LED_PIN = 25;   
const uint accel_gyro_addr = 0x68, mag_addr = 0x0C; //This the i2c slave data address of the mpu-9250

void mpu_reset() //This functions resets the sensor, 
{
    uint8_t buf[] = {0x6B, 0x00}; //This is a data buffer, where the first number is the register and the second number is the data that is gonna be written to said register
    i2c_write_blocking(i2c_default, accel_gyro_addr, buf, 2, false); //This functions is described in the "hardware/i2c" library, 

    uint8_t buff[] = {0x37, 0x02};
    i2c_write_blocking(i2c_default, accel_gyro_addr, buff, 2, false);

    uint8_t bufff[] = {0x0A, 0x16};
    i2c_write_blocking(i2c_default, mag_addr, bufff, 2, false);
}

void mpu_raw_read(int16_t accel[3], int16_t gyro[3], int16_t mag[3])
{
    uint8_t buffer[6];

    uint8_t val = 0x3B; //This is the start register of the accel meter

    i2c_write_blocking(i2c_default, accel_gyro_addr, &val, 1, true); //True to keep master control of the i2c 
    i2c_read_blocking(i2c_default, accel_gyro_addr, buffer, 6, false); //Here the data is written to the buffer variable, and the register is self counting

    for(int i = 0; i < 3; i++)
    {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    val = 0x43;

    i2c_write_blocking(i2c_default, accel_gyro_addr, &val, 1, true);
    i2c_read_blocking(i2c_default, accel_gyro_addr, buffer, 6, false);

    for(int i = 0; i < 3; i++)
    {
       gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    //val = 0x03;

    //i2c_write_blocking(i2c_default, mag_addr, &val, 1, true);

    uint8_t ST1;
    while(!(ST1 & 0x01))
    {
        i2c_read_blocking(i2c_default, mag_addr, &ST1, 1, true);
    }

    i2c_read_blocking(i2c_default, mag_addr, buffer, 6, false);

    for(int i = 0; i < 3; i++)
    {
        mag[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }
}

int main()
{ 
    stdio_init_all();
 
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    //gpio_init(LED_PIN);
    //gpio_set_dir(LED_PIN, GPIO_OUT);

    mpu_reset();

    int16_t acceleration[3], gyro[3], magneto[3];
    
    while(1)
    {
        /*gpio_put(LED_PIN,0);
        sleep_ms(250);
        gpio_put(LED_PIN,1);
        puts("Bruv\n");
        sleep_ms(1000);*/

        //gpio_put(LED_PIN, 0);

        mpu_raw_read(acceleration, gyro, magneto);

        printf("Acc. X = %d, Y = %d, Z = %d\n", acceleration[0], acceleration[1], acceleration[2]);
        printf("Gyro. X = %d, Y = %d, Z = %d\n", gyro[0], gyro[1], gyro[2]);
        printf("Mag. X = %d, Y = %d, Z = %d\n", magneto[0], magneto[1], magneto[2]);

        sleep_ms(100);

        //gpio_put(LED_PIN, 1);
    }
}
