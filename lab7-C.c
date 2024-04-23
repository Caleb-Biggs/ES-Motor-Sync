//Caleb's code here
#include <stdio.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "motor.h"
#include "pwm.h"


const uint8_t LED_PIN = 25;


void motor_test (void* notUsed);
void heartbeat(void* notUsed);


int main(){
	gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);
	motor_init();


	xTaskCreate(heartbeat, "LED_Task", 256, NULL, 1, NULL);
	xTaskCreate(motor_test, "motor", 256, NULL, 2, NULL);

	vTaskStartScheduler();
}


void motor_test(void* notUsed){
	while(1){
		motor_move(1000);
		vTaskDelay(5);
	}
}


void led_control(bool isOn, uint8_t whichLED){
    gpio_put(whichLED, isOn);
}


void flash_LED(int count, int freq, uint8_t whichLED){
    int time = 500/freq;
    for(int i = 0; i < count; i++){
        led_control(1, whichLED);
        vTaskDelay(time);
        led_control(0, whichLED);
        vTaskDelay(time);
    }
}


void heartbeat(void* notUsed){
    for(int count = 0;; count++) {
        printf("tick <%i>\n", count);
        flash_LED(1, 1, LED_PIN);
    }
}