#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h" 
#include "pico/binary_info.h" 
#include "hardware/i2c.h"
#include "hardware/spi.h"

#define GYRO_SENS 65.5   //FS_SEL{0,1,2,3}  = 131, 65.5, 32.8, 16.4
#define ACC_SENS 8192.0  //AFS_SEL{0,1,2,3} = 16384, 8192, 4096, 2048

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

void mpu_setup(uint8_t SDA_PIN, uint8_t SCL_PIN)
{
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    mpu_reset();
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

void complementary_filter(int16_t accel[3], int16_t gyro[3], float dt, float *pitch, float *roll)
{
    float pitch_acc, roll_acc;

    *pitch += ((float)gyro[0]/GYRO_SENS) * dt;
    *roll  -= ((float)gyro[1]/GYRO_SENS) * dt; 

    int16_t f_mag_approx = abs(accel[0]) + abs(accel[1]) + abs(accel[2]);
    if(f_mag_approx > 8192 && f_mag_approx < 32768) //0.5g to 2g
    {
        pitch_acc = atan2f((float)accel[1], (float)accel[2]) * 180 / M_PI;
        *pitch = *pitch * 0.98 + pitch_acc * 0.02;

        roll_acc = atan2f((float)accel[0], (float)accel[2]) * 180 / M_PI;
        *roll = *roll * 0.98 + roll_acc * 0.02;
    }
}

// void mpu9250_read_raw_gyro(int16_t gyro[3])
// {
//     uint8_t buffer[6];
//     uint8_t val = 0x43;

//     i2c_write_blocking(i2c_default, accel_gyro_addr, &val, 1, true);
//     i2c_read_blocking(i2c_default, accel_gyro_addr, buffer, 6, false);

//     for(int i = 0; i < 3; i++)
//     {
//        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
//     }
// }



void calibrate_gyro(double gyroCal[3], int loop)  //Used to calibrate the gyro. The gyro must be still while calibration happens
{
    int16_t accel[3], gyro[3], mag[3];
    for (int8_t i = 0; i < loop; i++)
    {
        mpu_raw_read(accel, gyro, mag);
        gyroCal[0] = (double)(gyroCal[0] + gyro[0]);
        gyroCal[1] = (double)(gyroCal[1] + gyro[1]);
        gyroCal[2] = (double)(gyroCal[2] + gyro[2]);

        // printf("cal:\n%f\n%f\n%f\n", gyroCal[0], gyroCal[1], gyroCal[2]);
        // printf("gyro:\n%d\n%d\n%d\n", gyro[0], gyro[1], gyro[2]);
        // sleep_ms(500);
    }
    gyroCal[0] /= (double)loop;
    gyroCal[1] /= (double)loop;
    gyroCal[2] /= (double)loop;
    // printf("test:\n%f\n%f\n%f\n", gyroCal[0], gyroCal[1], gyroCal[2]);
}



// int main()
// { 
//     stdio_init_all();
 
//     i2c_init(i2c_default, 400 * 1000);
//     gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
//     gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
//     gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
//     gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

//     //gpio_init(LED_PIN);
//     //gpio_set_dir(LED_PIN, GPIO_OUT);

//     mpu_reset();

//     int16_t acceleration[3], gyro[3], magneto[3];
    
//     while(1)
//     {
//         /*gpio_put(LED_PIN,0);
//         sleep_ms(250);
//         gpio_put(LED_PIN,1);
//         puts("Bruv\n");
//         sleep_ms(1000);*/

//         //gpio_put(LED_PIN, 0);

//         mpu_raw_read(acceleration, gyro, magneto);

//         printf("Acc. X = %d, Y = %d, Z = %d\n", acceleration[0], acceleration[1], acceleration[2]);
//         printf("Gyro. X = %d, Y = %d, Z = %d\n", gyro[0], gyro[1], gyro[2]);
//         printf("Mag. X = %d, Y = %d, Z = %d\n", magneto[0], magneto[1], magneto[2]);

//         sleep_ms(100);

//         //gpio_put(LED_PIN, 1);
//     }
// }

