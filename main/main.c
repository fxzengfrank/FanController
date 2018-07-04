#include <stdio.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_attr.h"
#include "soc/rtc.h"
#include "driver/mcpwm.h"
#include "driver/pcnt.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#define GPIO_PWM0A_OUT 2       //Set GPIO 19 as PWM0A
#define GPIO_PWM0B_OUT 21       //Set GPIO 18 as PWM0B

#define PCNT_TEST_UNIT PCNT_UNIT_0
#define PCNT_INPUT_SIG_IO 5     //Set GPIO 4 as PCNT signal input
#define PCNT_INPUT_CTRL_IO 22    //Set GPIO 5 as PCNT control input

static void pcnt_example_init(void)
{
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = PCNT_INPUT_SIG_IO,
        .ctrl_gpio_num = PCNT_INPUT_CTRL_IO,
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_TEST_UNIT,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DIS,
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_KEEP,
        .counter_h_lim = 1024,
        .counter_l_lim = 0,
    };
    pcnt_unit_config(&pcnt_config);
    pcnt_counter_pause(PCNT_TEST_UNIT);
    pcnt_counter_clear(PCNT_TEST_UNIT);
    pcnt_counter_resume(PCNT_TEST_UNIT);
}

static void mcpwm_example_gpio_initialize()
{
    printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
}

static void gpio_test_signal(void *arg)
{
    printf("intializing test signal...\n");
    gpio_config_t gp;
    gp.intr_type = GPIO_INTR_DISABLE;
    gp.mode = GPIO_MODE_OUTPUT;
    gp.pin_bit_mask = GPIO_SEL_12;  // GPIO 12
    gpio_config(&gp);
    while (1) {
        gpio_set_level(GPIO_NUM_12, 1);		//Set high
        vTaskDelay(10 / portTICK_RATE_MS);	//delay of 10ms
        gpio_set_level(GPIO_NUM_12, 0);		//Set low
        vTaskDelay(10 / portTICK_RATE_MS);	//delay of 10ms
    }
}

static void mcpwm_example_config(void *arg)
{
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();

    //2. initialize mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 101;    //frequency = 1000Hz
    pwm_config.cmpr_a = 60.1;       //duty cycle of PWMxA = 50.0%
    pwm_config.cmpr_b = 30.1;       //duty cycle of PWMxb = 10.0%
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);   //Configure PWM0A & PWM0B with above settings

    vTaskDelete(NULL);
}

void app_main()
{
    printf("Testing MCPWM...\n");
    xTaskCreate(mcpwm_example_config, "mcpwm_example_config", 4096, NULL, 5, NULL);
    //gpio_test_signal(NULL);
    pcnt_example_init();
    int16_t count = 0;
    while (1) {
        pcnt_get_counter_value(PCNT_TEST_UNIT, &count);
        pcnt_counter_clear(PCNT_TEST_UNIT);
        printf("Counter: %d\n", count);
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

