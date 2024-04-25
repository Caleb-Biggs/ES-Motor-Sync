#include <stdio.h>
#include <stdint.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include "motor.h"
#include "pwm.h"

const uint8_t LED_PIN = 25;



void hardware_init(void)
{
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
}

void motor_test(void* notUsed){
	while(1){
	    motor_move(1000);
        motor_get_position();
        vTaskDelay(50);
        motor_move(1000);
        motor_get_position();
	}
}


void heartbeat(void * notUsed)
{
    while (true) {
        gpio_put(LED_PIN, 1);
        vTaskDelay(50);
        gpio_put(LED_PIN, 0);
        vTaskDelay(50);

    }
}

int main()
{
    stdio_init_all();
    hardware_init();
    motor_init();
    xTaskCreate(heartbeat, "LED_Task", 256, NULL, 1, NULL);
    xTaskCreate(motor_test, "Motor_Task",256,NULL,2,NULL);
    vTaskStartScheduler();

    while(1)
    {};   
}