#include <stdio.h>
#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "motor.h"
#include "pwm.h"
#include "lab7.h"


/****************\
|Master Functions|
\****************/
#if 1
//8 Got no errors with no delay between commands in the master and
//Set to 80 for debugging purposes
#define SLEEP 80
#define send_bit(pin, bit){\
	sleep_us(SLEEP);\
	gpio_put((pin), (bit));\
}


void master_spi(void* notUsed);

void motor_test(void* notUsed){
	_pidPositionServo();
	// motor_move(25);
	while(1) vTaskDelay(1000);
}


int main(){
    stdio_init_all();
    hardware_init();
    motor_init();
    xTaskCreate(heartbeat, "LED_Task", 256, NULL, 1, NULL);
    // xTaskCreate(motor_test, "Motor_Task",256,NULL,2,NULL);
    xTaskCreate(master_spi, "Master SPI", 256, NULL, 3, NULL);
    vTaskStartScheduler();

    while(1);
}


uint8_t read_byte(){
    uint8_t output = 0;
    for(int i = 0; i < 8; i++){
        send_bit(CLK_PIN, 1);
        // gpio_put(CLK_PIN, 1);
        uint8_t a = gpio_get(MISO_PIN);
        output <<= 1;
        output |= a;
        send_bit(CLK_PIN, 0);
        sleep_us(SLEEP);
    }
    return output;
}


void send_byte(uint8_t byte){
    for(int i = 0; i < 8; i++){
        send_bit(MOSI_PIN, 0);
        send_bit(MOSI_PIN, byte&0x80);
        byte <<= 1;
        send_bit(CLK_PIN, 1);
        send_bit(CLK_PIN, 0);
    }
}


uint8_t send_command(uint8_t byte1, uint8_t byte2){
    uint8_t output;
    send_bit(CS_PIN, 0);
    send_byte(byte1);
    if((byte1&0x0F) == READ_COM)
    	output = read_byte();
    else send_byte(byte2);
    send_bit(CS_PIN, 1);
    return output;
}


void master_spi(void* notUsed){
	while(1){
		send_command(LED_REG|WRITE_COM, 0xF0);
        printf("Wrote 1s: %b\n", send_command(LED_REG|READ_COM,0));
        vTaskDelay(500);
        send_command(LED_REG|WRITE_COM, 0x00);
        printf("Wrote 0s: %b\n", send_command(LED_REG|READ_COM,0));
        vTaskDelay(500);
	}
}


void init_helper(uint8_t pin, bool dir){
    gpio_init(pin);
    gpio_set_dir(pin, dir);
}


void hardware_init(){
	init_helper(LED_PIN, GPIO_OUT);
	init_helper(CS_PIN, GPIO_OUT);
    init_helper(CLK_PIN, GPIO_OUT);
    init_helper(MOSI_PIN, GPIO_OUT);
    init_helper(MISO_PIN, GPIO_IN);
    gpio_put(CS_PIN, 1);
}


void heartbeat(void * notUsed){
    while(1){
        gpio_put(LED_PIN, 1);
        vTaskDelay(250);
        gpio_put(LED_PIN, 0);
        vTaskDelay(750);
    }
}




/****************\
| Slave Functions|
\****************/
#else

#define DEBUG 0
uint8_t test;

static QueueHandle_t instructionQueue;
static QueueHandle_t register_queue;

int main(){
	stdio_init_all();
    hardware_init();
    // motor_init();

    instructionQueue = xQueueCreate(256, 1);
    register_queue = xQueueCreate(256, sizeof(reg_update));

    xTaskCreate(heartbeat, "LED_Task",256,NULL,1,NULL);

    //This task handles all of the writes to the registers in a single queue
    xTaskCreate(update_registers, "Update Registers",256,NULL,2,NULL);

    //Contains the state machine that controlls the SPI communication
    xTaskCreate(slave_state, "Slave State",256,NULL,3,NULL);

    vTaskStartScheduler();

    while(1);
}


void init_helper(uint8_t pin, bool dir, bool isInterrupt){
    gpio_init(pin);
    gpio_set_dir(pin, dir);
    if(!isInterrupt) return;
    gpio_set_irq_enabled_with_callback(pin, GPIO_IRQ_EDGE_FALL, true, &gpio_int_callback);
    gpio_set_irq_enabled_with_callback(pin, GPIO_IRQ_EDGE_RISE, true, &gpio_int_callback);
}


void hardware_init(){
	init_helper(LED_PIN, GPIO_OUT, false);
	init_helper(CS_PIN, GPIO_IN, true);
    init_helper(CLK_PIN, GPIO_IN, true);
    init_helper(MOSI_PIN, GPIO_IN, false);
    init_helper(MISO_PIN, GPIO_OUT, false);
}


void heartbeat(void * notUsed){
    while(1){
        gpio_put(LED_PIN, 1);
        vTaskDelay(750);
        gpio_put(LED_PIN, 0);
        vTaskDelay(250);
    }
}


////////////////////
//State Machine/////

void to_send(int8_t mode, uint8_t val);


int8_t get_byte(uint8_t* byte){
	uint8_t buffer;
	for(int i = 0; i < 8; i++){
		xQueueReceive(instructionQueue, &buffer, portMAX_DELAY);
		buffer &= 0x0F;
		if(buffer > 1) return ERR_GET_BYTE;
		*byte = ((*byte)<<1)|buffer;
		xQueueReceive(instructionQueue, &buffer, portMAX_DELAY);
	}
	return 0;
}


int8_t idle_state(STATE* state){
	if(DEBUG) printf("%s\n", __func__);
	uint8_t buffer;
	xQueueReceive(instructionQueue, &buffer, portMAX_DELAY);
	if(buffer != CSL_INST) return ERR_EXP_CS_LO;
	*state = COMMAND_S;
	return 0;
}


int8_t command_state(uint8_t* byte_1, STATE* state){
	if(DEBUG) printf("%s\n", __func__);
	int8_t error = get_byte(byte_1);
	if(error) return ERR_GET_COMM;
	switch((*byte_1)&0x0F){
		case READ_COM: *state = READ_S; break;
		case WRITE_COM: *state = WRITE_S; break;
		case TESTING_COM: while(1); //For testing watchdog
		default: return ERR_GET_COMM;
	}
	return 0;
}


int8_t read_state(uint8_t byte_1, STATE* state){
	if(DEBUG) printf("%s\n", __func__);
	uint8_t buffer = CLKL_INST;
	uint8_t toSend = registers(READ_REG, byte_1, 0);
	for(int i = 0; i < 8; i++){
		gpio_put(MISO_PIN, toSend&0x80);
		toSend <<= 1;
		if(buffer != CLKL_INST) return ERR_SEND_REG;
		xQueueReceive(instructionQueue, &buffer, portMAX_DELAY);
		xQueueReceive(instructionQueue, &buffer, portMAX_DELAY);
	}
	// printf("Wrote %b\n", registers(READ_REG, byte_1, 0));
	*state = DONE_S;
	return 0; 
}


int8_t write_state(uint8_t byte_1, STATE* state){
	if(DEBUG) printf("%s\n", __func__);
	uint8_t byte_2;
	int8_t error = get_byte(&byte_2);
	if(error) return ERR_WRT_DATA;
	registers(OVERWRITE_REG, byte_1, byte_2);
	*state = DONE_S;
	return 0;
}


void done_state(int8_t* error, STATE* state){
	if(DEBUG) printf("%s\n", __func__);
	uint8_t buffer;
	do{ xQueueReceive(instructionQueue, &buffer, portMAX_DELAY);
	} while(buffer != CSH_INST);
	*error = 0;
	*state = IDLE_S;
}


void error_check(int8_t* code, STATE* state){
	//TODO: Remove unused error codes and give them more useful messages
	switch(*code){
        case 0: break;
        case ERR_GET_COMM: printf("ERR_GET_COMM\n"); break;
        case ERR_WRT_DATA: printf("ERR_WRT_DATA\n"); break;
        case ERR_REG_VAL: printf("ERR_REG_VAL\n"); break;
        case ERR_COMM_EX: printf("ERR_COMM_EX\n"); break;
        case ERR_SHLDNT_RCH: printf("ERR_SHLDNT_RCH\n"); break;
        case ERR_SEND_REG: printf("ERR_SEND_REG\n"); break;
        case ERR_EXP_CS_LO: printf("ERR_EXP_CS_LO\n"); break;
        case ERR_EXP_CS_HI: printf("ERR_EXP_CS_HI\n"); break;
        case ERR_EXP_CLK_LO: printf("ERR_EXP_CLK_LO\n"); break;
		case ERR_EXP_CLK_HI: printf("ERR_EXP_CLK_HI\n"); break;
		case ERR_GET_BYTE: printf("ERR_GET_BYTE\n"); break;
        default: printf("invalid error code\n"); break;
    }
    *state = DONE_S;
}


void slave_state(void* notUsed){
	enum STATE state = IDLE_S;
	uint8_t byte_1;
	int8_t error = 0;
	while(1){
		switch(state){
			case IDLE_S: error = idle_state(&state); break;
			case COMMAND_S: error = command_state(&byte_1, &state); break;
			case READ_S: error = read_state(byte_1, &state); break;
			case WRITE_S: error = write_state(byte_1, &state); break;
			case DONE_S: done_state(&error, &state); break;
		}
		if(error) error_check(&error, &state);
	}
}


////////////////////
//Registers/////////


uint8_t registers(REG_ARG mode, uint8_t reg, uint8_t val){
	static uint8_t registers[3] = {0};
    uint8_t index = ((reg)>>4)-1;
    if(mode == READ_REG) return registers[index];
    reg_update update = {&(registers[index]), val, mode};
    xQueueSendToBack(register_queue, &update, portMAX_DELAY);
    return 0;
}


void update_registers(void* notUsed){
	reg_update update;
	while(1){
		xQueueReceive(register_queue, &update, portMAX_DELAY);
		switch(update.arg){
			case OVERWRITE_REG: *(update.reg) = update.val; break;
			case OR_REG: *(update.reg) |= update.val; break;
			case AND_REG: *(update.reg) &= update.val; break;
			default: break;
		}
	}
}


////////////////////
//Interrupts////////

uint8_t test = 1;
void gpio_int_callback(uint gpio, uint32_t events_unused) {
    uint8_t state;
    BaseType_t yield = pdFALSE;
    switch(gpio){
        case CS_PIN:
            state = (gpio_get(CS_PIN)) ? CSH_INST : CSL_INST;
            xQueueSendToBackFromISR(instructionQueue, &state, NULL);
            break;
        case CLK_PIN:
            if(gpio_get(CLK_PIN)){ //CLK High
            	uint8_t val = CLKH_INST|gpio_get(MOSI_PIN);
            	xQueueSendToBackFromISR(instructionQueue, &val, &yield);
            	if(yield) taskYIELD();
            } else{
            	xQueueSendToBackFromISR(instructionQueue, &CLKL_INST, &yield);
            	if(yield) taskYIELD();
            }
            break;
    }
}


/****************\
|Shared Functions|
\****************/
#endif