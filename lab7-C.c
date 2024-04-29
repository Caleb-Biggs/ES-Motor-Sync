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
#if 0

void motor_test(void* notUsed){
	_pidPositionServo();
}


int main(){
    stdio_init_all();
    hardware_init();
    motor_init();
    xTaskCreate(heartbeat, "LED_Task", 256, NULL, 1, NULL);
    // xTaskCreate(motor_test, "Motor_Task",256,NULL,2,NULL);
    vTaskStartScheduler();

    while(1);
}


void hardware_init(){
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
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

typedef enum STATE {IDLE, COMMAND, READ, WRITE, DONE} STATE;

static QueueHandle_t instructionQueue;

void slave_state(void* notUsed);

typedef enum REG_ARG { READ_REG, OVERWRITE_REG, AND_REG, OR_REG } REG_ARG;
typedef struct REG_UPDATE { uint8_t* reg; uint8_t val; REG_ARG arg; } reg_update;
static QueueHandle_t register_queue;
void update_registers(void* notUsed);
uint8_t registers(REG_ARG mode, uint8_t reg, uint8_t val);

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


int8_t get_byte(uint8_t* byte){
	uint8_t buffer;
	for(int i = 0; i < 8; i++){
		xQueueReceive(instructionQueue, &buffer, portMAX_DELAY);
		xQueueReceive(instructionQueue, &buffer, portMAX_DELAY);
		if(buffer > 1) return ERR_GET_BYTE;
		*byte = ((*byte)<<1)|buffer;
		xQueueReceive(instructionQueue, &buffer, portMAX_DELAY);
	}
	gpio_put(MISO_PIN, registers(READ_REG, *byte, 0)&0x80);
	return 0;
}


int8_t idle_state(STATE* state){
	printf("%s\n", __func__);
	uint8_t buffer;
	xQueueReceive(instructionQueue, &buffer, portMAX_DELAY);
	if(buffer != CSL_INST) return ERR_EXP_CS_LO;
	*state = COMMAND;
	return 0;
}


int8_t command_state(uint8_t* byte_1, STATE* state){
	printf("%s\n", __func__);
	int8_t error = get_byte(byte_1);
	if(error) return ERR_GET_COMM;
	switch((*byte_1)&0x0F){
		case READ_COM: *state = READ; break;
		case WRITE_COM: *state = WRITE; break;
		case TESTING_COM: while(1); //For testing watchdog
		default: return ERR_GET_COMM;
	}
	return 0;
}


int8_t read_state(uint8_t byte_1, STATE* state){
	printf("%s\n", __func__);
	uint8_t buffer;
	uint8_t toSend = registers(READ_REG, byte_1, 0)<<1;
	// uint8_t toSend = 0b01101101;
	for(int i = 0; i < 8; i++){
		xQueueReceive(instructionQueue, &buffer, portMAX_DELAY);
		xQueueReceive(instructionQueue, &buffer, portMAX_DELAY);
		xQueueReceive(instructionQueue, &buffer, portMAX_DELAY);
		gpio_put(MISO_PIN, toSend&0x80);
		if(buffer != CLKL_INST) return ERR_SEND_REG;
		toSend <<= 1;
	}
	printf("Wrote %b\n", registers(READ_REG, byte_1, 0));
	*state = DONE;
	return 0; 
}


int8_t write_state(uint8_t byte_1, STATE* state){
	printf("%s\n", __func__);
	uint8_t byte_2;
	int8_t error = get_byte(&byte_2);
	if(error) return ERR_WRT_DATA;
	registers(OVERWRITE_REG, byte_1, byte_2);
	*state = DONE;
	return 0;
}


void done_state(int8_t* error, STATE* state){
	printf("%s\n", __func__);
	uint8_t buffer;
	do{ xQueueReceive(instructionQueue, &buffer, portMAX_DELAY);
	} while(buffer != CSH_INST);
	*error = 0;
	*state = IDLE;
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
    *state = DONE;
}


void slave_state(void* notUsed){
	enum STATE state = IDLE;
	uint8_t byte_1;
	int8_t error = 0;
	while(1){
		switch(state){
			case IDLE: error = idle_state(&state); break;
			case COMMAND: error = command_state(&byte_1, &state); break;
			case READ: error = read_state(byte_1, &state); break;
			case WRITE: error = write_state(byte_1, &state); break;
			case DONE: done_state(&error, &state); break;
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


void gpio_int_callback(uint gpio, uint32_t events_unused) {
    uint8_t state;
    switch(gpio){
        case CS_PIN:
            state = (gpio_get(CS_PIN)) ? CSH_INST : CSL_INST;
            xQueueSendToBackFromISR(instructionQueue, &state, NULL);
            break;
        case CLK_PIN:
            state = (gpio_get(CLK_PIN)) ? CLKH_INST : CLKL_INST;
            uint8_t val = gpio_get(MOSI_PIN);
            xQueueSendToBackFromISR(instructionQueue, &state, NULL);
            if(state == CLKH_INST) xQueueSendToBackFromISR(instructionQueue, &val, NULL);
            break;
    }
}


/****************\
|Shared Functions|
\****************/
#endif