/* STANDARD LIBRARY */
#include <stdio.h>
#include <math.h>

/* PICO INCLUDES */
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

/* USER DEFINED */
#include "../gpio/PWMGpio.h"
#include "../gpio/Gpio.h"
#include "../mpu/mpu.h"

typedef struct {
    double dt;
    double max;
    double min;
    double Kp;
    double Ki;
    double Kd;
    double pre_error;
    double integral;
}PID; 

double pid_calculate(PID* controller, double reference, double prev)
{
    double error = reference - prev;
    
    /* Proportional */
    double Pout = controller->Kp * error;

    /* Integral */
    controller->integral += error * controller->dt;
    double Iout = controller->Ki * controller->integral;

    /* Derivative */
    if (controller->dt == 0.0)
    {
        printf("Error: dt == 0.0\n");
        return -1;
    }
    double derivative = (error - controller->pre_error) / controller->dt;
    double Dout = controller->Kd * derivative;

    double output = Pout + Iout + Dout;

    if(output > controller->max)
        output = controller->max;
    else if(output < controller->min)
        output = controller->min;

    controller->pre_error = error;

    //Convert output to duty cycle range:
    // double newOut = ((output - controller->min)*(100 - 0))/(controller->max - controller->min) + 0;

    return output;
}

uint32_t main()
{
    stdio_init_all();
    sleep_ms(2000);

    PID controller = {
        .dt = 0.1,          //Initial start guess
        .max = 100,
        .min = -100,
        .Kp = 2,
        .Ki = 0,//0.01,//.01,
        .Kd = 0.5,//.5,
        .pre_error = 0,
        .integral = 0
    };
    PID* controller_p = &controller;

    PWMGpio front_right = {
        .pin_number = 12,           //GPIO number
        .pwm_chan   = PWM_CHAN_A,   //PWM Channel
        .pwm_range  = 65535,        //PWM range
        .slice_num  = 6             //Slice number is found in createPWM, ie default 0
    };
    PWMGpio back_left = {
        .pin_number = 13,           //GPIO number
        .pwm_chan   = PWM_CHAN_B,   //PWM Channel
        .pwm_range  = 65535,        //PWM range
        .slice_num  = 6             //Slice number is found in createPWM, ie default 0
    };
    PWMGpio back_right = {
        .pin_number = 14,           //GPIO number
        .pwm_chan   = PWM_CHAN_A,   //PWM Channel
        .pwm_range  = 65535,        //PWM range
        .slice_num  = 7             //Slice number is found in createPWM, ie default 0
    };
    PWMGpio front_left = {
        .pin_number = 15,           //GPIO number
        .pwm_chan   = PWM_CHAN_B,   //PWM Channel
        .pwm_range  = 65535,        //PWM range
        .slice_num  = 7             //Slice number is found in createPWM, ie default 0
    };
    PWMGpio* front_right_p = &front_right;   //Create pointer to object
    PWMGpio* back_left_p = &back_left;       //Create pointer to object
    PWMGpio* back_right_p = &back_right;     //Create pointer to object
    PWMGpio* front_left_p = &front_left;     //Create pointer to object
    createPWM(front_right_p, 50);            //Create the PWM signal
    createPWM(back_left_p, 50);              //Create the PWM signal
    createPWM(back_right_p, 50);             //Create the PWM signal
    createPWM(front_left_p, 50);             //Create the PWM signal


    mpu_setup(8, 9);
    int16_t acceleration[3], gyro[3], magneto[3];

    uint16_t duty_cycle = 100;
    absolute_time_t prev_time, time;
    prev_time = get_absolute_time();

    double gyroCal[3];
    calibrate_gyro(gyroCal, 100);
    printf("Gyro calibration done.\n");
    printf("gyroX: %f\ngyroY: %f\ngyroZ: %f\n", gyroCal[0], gyroCal[1], gyroCal[2]);
    sleep_ms(2000);
    float pitch = 0, roll = 0;
    while(true)
    {
        time = get_absolute_time();
        controller_p->dt = (double) absolute_time_diff_us(prev_time, time)/1000000.0;
        // controller_p->dt = (double) to_ms_since_boot(time-prev_time)/1000;

        /* ----------------------------------------------------------- */
        mpu_raw_read(acceleration, gyro, magneto);
        gyro[0] = gyro[0]-gyroCal[0];
        gyro[1] = gyro[1]-gyroCal[1];
        gyro[2] = gyro[2]-gyroCal[2];

        complementary_filter(acceleration, gyro, controller_p->dt, &pitch, &roll);

        printf("roll: %f\npitch: %f\n", roll, pitch);
        // printf("gyroX: %d\ngyroY: %d\ngyroZ: %d\n", gyro[0], gyro[1], gyro[2]);
        // printf("gyroX: %f\ngyroY: %f\ngyroZ: %f\n", (double)(gyro[0] - gyroCal[0]), (double)(gyro[1] - gyroCal[1]), (double)(gyro[2] - gyroCal[2]));
        // int16_t forceMagApprox = abs(acceleration[0]) + abs(acceleration[1]) + abs(acceleration[2]);
        // printf("x: %d\ny: %d\nz: %d\nforceMagApprox: %d\n", acceleration[0], acceleration[1], acceleration[2], forceMagApprox);


        // double pitch = 180 * atan2(acceleration[0], sqrt(acceleration[1]*acceleration[1] + acceleration[2]*acceleration[2]))/M_PI;
        // double roll  = 180 * atan2(acceleration[1], sqrt(acceleration[0]*acceleration[0] + acceleration[2]*acceleration[2]))/M_PI;

        // double mag_x = magneto[0]*cos(pitch) + magneto[1]*sin(roll)*sin(pitch) + magneto[2]*cos(roll)*sin(pitch);
        // double mag_y = magneto[1]*cos(roll) - magneto[2]*sin(roll);
        // double yaw = 180 * atan2(-magneto[1],magneto[0])/M_PI;

        // printf("roll: %f\npitch: %f\nyaw: %f\n", roll, pitch, yaw);

        // printf("gyroX: %f\ngyroY: %f\ngyroZ: %f\n", gyro[0]-gyroCal[0], gyro[1]-gyroCal[1], gyro[2]-gyroCal[2]);

        double output = pid_calculate(controller_p, 90, roll);
        setPWM(front_right_p, output, NORMAL);
        printf("pid: %f\n", output);
        /* ----------------------------------------------------------- */

        printf("time: %f\n", controller_p->dt);
        prev_time = time;
        sleep_ms(100);
    }


    return 0;
}


        // setPWM(front_right_p, duty_cycle, NORMAL);
        // setPWM(back_left_p, duty_cycle, NORMAL);
        // setPWM(back_right_p, duty_cycle, NORMAL);
        // setPWM(front_left_p, duty_cycle, NORMAL);

        // mpu_raw_read(acceleration, gyro, magneto);
        // double xAngle = atan(acceleration[0]/sqrt(acceleration[1]*acceleration[1]+acceleration[2]*acceleration[2]));
        // double yAngle = atan(acceleration[1]/sqrt(acceleration[0]*acceleration[0]+acceleration[2]*acceleration[2]));
        // printf("angleX: %f\nangleY: %f\n", xAngle, yAngle);
        // printf("Acc. X = %d, Y = %d, Z = %d\n", acceleration[0], acceleration[1], acceleration[2]);
        // printf("Gyro. X = %d, Y = %d, Z = %d\n", gyro[0], gyro[1], gyro[2]);
        // printf("Mag. X = %d, Y = %d, Z = %d\n", magneto[0], magneto[1], magneto[2]);
        // printf("time: %d\n", to_ms_since_boot(time-prev_time));