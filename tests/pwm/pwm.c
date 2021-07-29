/* STANDARD LIBRARY */
#include <stdio.h>

/* PICO INCLUDES */
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

/* USER DEFINED */
#include "../gpio/PWMGpio.h"
#include "../gpio/Gpio.h"

int main()
{
    stdio_init_all();

    PWMGpio pwm12 = {
        .pin_number = 12,           //GPIO number
        .pwm_chan   = PWM_CHAN_A,   //PWM Channel
        .pwm_range  = 65535,         //PWM range
        .slice_num  = 6             //Slice number is found in createPWM, ie default 0
    };
    PWMGpio pwm13 = {
        .pin_number = 13,           //GPIO number
        .pwm_chan   = PWM_CHAN_B,   //PWM Channel
        .pwm_range  = 65535,         //PWM range
        .slice_num  = 6             //Slice number is found in createPWM, ie default 0
    };
    PWMGpio pwm14 = {
        .pin_number = 14,           //GPIO number
        .pwm_chan   = PWM_CHAN_A,   //PWM Channel
        .pwm_range  = 65535,         //PWM range
        .slice_num  = 7             //Slice number is found in createPWM, ie default 0
    };
    PWMGpio pwm15 = {
        .pin_number = 15,           //GPIO number
        .pwm_chan   = PWM_CHAN_B,   //PWM Channel
        .pwm_range  = 65535,         //PWM range
        .slice_num  = 7             //Slice number is found in createPWM, ie default 0
    };
    PWMGpio* pwm12p = &pwm12;       //Create pointer to object
    PWMGpio* pwm13p = &pwm13;       //Create pointer to object
    PWMGpio* pwm14p = &pwm14;       //Create pointer to object
    PWMGpio* pwm15p = &pwm15;       //Create pointer to object
    createPWM(pwm12p, 50);              //Create the PWM signal
    createPWM(pwm13p, 50);              //Create the PWM signal
    createPWM(pwm14p, 50);              //Create the PWM signal
    createPWM(pwm15p, 50);              //Create the PWM signal

    

    // uint32_t pwm_val = 100;
    // uint32_t i = 0;
    // initESC(pwm12p);
    // initESC(pwm13p);
    // initESC(pwm14p);
    // initESC(pwm15p);

    float val = 0;
    while(true)
    {
        setPWM(pwm12p, val);
        setPWM(pwm13p, val);
        setPWM(pwm14p, val);
        setPWM(pwm15p, val);
        // setPWM(pwm15p, 1);
        // sleep_ms(1000);
        // printf("test\n");
        // setPWM(pwm15p, 50);
        // sleep_ms(1000);
        // setPWM(pwm15p, 100);
        // sleep_ms(1000);
        // printf("count: %d\n", pwm_get_counter(pwm15p->slice_num));
        // while(pwm_val < 65500)
        // {
        //     setPWM(pwm15p, pwm_val);
        //     // setPWM(pwm_defaultLEDp, pwm_val);
            
        //     sleep_ms(1);
        //     pwm_val += 30;
        // }
        // while(pwm_val > 0)
        // {
        //     setPWM(pwm15p, pwm_val);
        //     // setPWM(pwm_defaultLEDp, pwm_val);
        //     sleep_ms(1);
        //     pwm_val -= 30;
        // }
        // printf("i: %d\n", i);
        // i++;
    }

    return 0;
}