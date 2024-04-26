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


extern double Kp, Ki, Kd;
void motor_test(void* notUsed){
	Kp = 1;
	Ki = 0;
	Kd = 0;
	motor_move(75);
	int previous_error = 0;
	int integral = 0;
	int setpoint = 5000;//
	int dt = 50;
	while(1){
		int measured_value = motor_get_position();
		int error = setpoint - measured_value;
		int proportional = error;
		integral = integral + error * dt;
		double derivative = (double)(error - previous_error) / dt;
		double output = Kp * proportional + Ki * integral + (double)Kd * derivative;
		previous_error = error;
		printf("Output = %lf\n", output);
		vTaskDelay(dt);
		setpoint = motor_get_position() + 5000;


		// printf("Pos = %i\n", motor_get_position());
	    // motor_move(1000);
        // motor_get_position();
        // vTaskDelay(5);
        // motor_move(1000);
        // motor_get_position();
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