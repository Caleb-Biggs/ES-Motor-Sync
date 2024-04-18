#include <stdio.h>
#include "hardware/gpio.h"
#include "pico/stdlib.h"


const uint8_t QUAD_A_PIN = 6;
const uint8_t QUAD_B_PIN = 7;
// const uint8_t MOTOR_A_PIN = 8;
// const uint8_t MOTOR_B_PIN = 9;
const uint8_t SW_PIN = 14;

const uint16_t SLEEP = 500;


int32_t pos = 0;


void hardware_init();


int main(){
	stdio_init_all();
    hardware_init();

    int32_t prevPos = 0;
    while(1){
    	sleep_ms(SLEEP);
    	int32_t delta = pos - prevPos;
    	prevPos = pos;
    	printf("pos = %i, RPM = %lf\n", pos, ((double)50/SLEEP)*delta);
    }
}


void gpio_int_callback(uint gpio, uint32_t events_unused){
	static int8_t A = 0;
	static int8_t B = 0;
	switch(gpio){
		case QUAD_A_PIN: pos -= ((B ^ (A = gpio_get(QUAD_A_PIN)))<<1)-1; break;
		case QUAD_B_PIN: pos += ((A ^ (B = gpio_get(QUAD_B_PIN)))<<1)-1; break;
		//WIP
		// case QUAD_A_PIN: pos += (B ^ (A = events_unused))+2;
		// case QUAD_B_PIN: pos -= (A ^ (B = events_unused))+2;
	}
}


void hardware_init(){
	gpio_init(QUAD_A_PIN);
    gpio_set_dir(QUAD_A_PIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(QUAD_A_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_int_callback);
    gpio_set_irq_enabled_with_callback(QUAD_A_PIN, GPIO_IRQ_EDGE_RISE, true, &gpio_int_callback);
    gpio_init(QUAD_B_PIN);
    gpio_set_dir(QUAD_B_PIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(QUAD_B_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_int_callback);
    gpio_set_irq_enabled_with_callback(QUAD_B_PIN, GPIO_IRQ_EDGE_RISE, true, &gpio_int_callback);

    gpio_init(SW_PIN);
    gpio_set_dir(SW_PIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(SW_PIN, GPIO_IRQ_EDGE_RISE, true, &gpio_int_callback);
    gpio_set_irq_enabled_with_callback(SW_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_int_callback);
    gpio_pull_up(SW_PIN);

    // gpio_init(MOTOR_A_PIN);
    // gpio_set_dir(MOTOR_A_PIN, GPIO_OUT);
    // gpio_init(MOTOR_B_PIN);
    // gpio_set_dir(MOTOR_B_PIN, GPIO_OUT);
}