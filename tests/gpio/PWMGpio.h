#ifndef PWMGPIO
#define PWMGPIO
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

typedef struct PWMGpio {
    const uint8_t pin_number;
    uint8_t pwm_chan;
    uint32_t pwm_range;
    uint8_t slice_num;
    uint32_t top;
} PWMGpio;

/**
 * @brief Sets up a GPIO as PWM with a desired frequency
 * @param pg Is a pointer to the PWMGpio struct containing various information regarding the pwm GPIO.
 * @param desired_freq Is the frequency the PWM should run at.
 * */
void createPWM(PWMGpio* pg, const uint32_t desired_freq)
{
    //Set GPIO functionality as PWM
    gpio_set_function(pg->pin_number, GPIO_FUNC_PWM);
    
    //Find PWM GPIO slice number
    pg->slice_num = pwm_gpio_to_slice_num(pg->pin_number);

    //Set frequency
    uint32_t f_sys = clock_get_hz(clk_sys);
    float divider = f_sys / 1000000UL;
    pwm_set_clkdiv(pg->slice_num, divider);
    pg->top = 1000000UL/desired_freq - 1;

    //Set the range of the PWM
	// pwm_set_wrap(pg->slice_num, pg->pwm_wrap);
    pwm_set_wrap(pg->slice_num, pg->top);

    //Set PWM to 0 when created
    pwm_set_chan_level(pg->slice_num, pg->pwm_chan, 0);

    //Enable PWM
    pwm_set_enabled(pg->slice_num, true);
}

void initESC(const PWMGpio* pg)
{
    /**
     * *Initialize ESC by setting the throttle range. First max throttle, highest was 70 for some reason.
     * After flashing the program to the drone the battery has to be connected shortly after and then this
     * function will run and initialize the ESC. The minimum throttle of the ESC is equivalent to 5% duty-
     * cycle and maximum throttle is equal to 10% dutycycle. */
    uint16_t duty_cycle = 10;   //Max throttle
    uint16_t level = (pg->top+1) * duty_cycle / 100 - 1; // calculate channel level from given duty cycle in %

    pwm_set_chan_level(pg->slice_num, pg->pwm_chan, level); 
    sleep_ms(10000);

    duty_cycle = 5; //Min throttle
    level = (pg->top+1) * duty_cycle / 100 - 1; // calculate channel level from given duty cycle in %
    pwm_set_chan_level(pg->slice_num, pg->pwm_chan, level); 

    sleep_ms(3000);
    printf("ESC calibration done!\n");
    //Now the duty-cycle/throttle can be freely set to anything.
}

/**
 * @brief Sets our own defined duty cycle (0-100%) of 5-10% equivalent to 1ms and 2ms, which is the min and max of ESC.
 * @param pg Is a pointer to the PWMGpio struct containing various information regarding the pwm GPIO.
 * @param duty_cycle Is our defined duty-cycle between 0-100%, which corresponds to a value between 1ms and 2ms.
 * */
void setPWM(const PWMGpio* pg, float duty_cycle)
{
    //Find a pulse-width (in ms) based on the duty-cycle:
    if(duty_cycle < 0.0f)
        duty_cycle = 0;
    else if(duty_cycle > 100.0f)
        duty_cycle = 100;
    float pw_ms = 5 + 0.05 * duty_cycle;    //start duty-cycle (5%) + 1% increase * "homemade dutycycle"

    // Set duty cycle
	uint16_t level = (pg->top+1) * pw_ms / 100 - 1; // calculate channel level from given duty cycle in %
	pwm_set_chan_level(pg->slice_num, pg->pwm_chan, level); 
}

#endif