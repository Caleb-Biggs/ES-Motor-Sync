#include <stdio.h>
#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "hardware/watchdog.h"
#include "hardware/structs/watchdog.h"
#include "motor.h"
#include "pwm.h"
#include "lab7.h"

//TODO: Fix Master LED behavior


/****************\
|Master Functions|
\****************/
#if 0

const uint8_t NEXT_POS = 0;
const uint8_t PREV_POS = 1;
const uint8_t STOP_POS = 2;

static QueueHandle_t positionQueue;
static QueueHandle_t SPIQueue;
static QueueHandle_t LEDQueue;
static QueueHandle_t SPIReadQueue;

typedef struct LED_BEHAVIOR { bool R, G, B; uint8_t freq; float duty; bool doStop; } led_behavior;


int main(){
    stdio_init_all();
    hardware_init();
    motor_init(0.8, 0, 0, true);

    positionQueue = xQueueCreate(256, sizeof(uint8_t));
    SPIQueue = xQueueCreate(256, sizeof(spi_send));
    SPIReadQueue = xQueueCreate(256, sizeof(uint8_t));
    LEDQueue = xQueueCreate(256, sizeof(led_behavior));

    xTaskCreate(heartbeat, "LED_Task", 256, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(ext_LED, "External LED", 256, NULL, tskIDLE_PRIORITY+1, NULL);
    xTaskCreate(monitor_slave, "Nanny", 256, NULL, tskIDLE_PRIORITY+1, NULL);
    xTaskCreate(motor_cycle, "Motor_Task",256,NULL,tskIDLE_PRIORITY+3,NULL);
    xTaskCreate(master_spi, "Master SPI", 256, NULL, tskIDLE_PRIORITY+4, NULL);
    xTaskCreate(switch_handler, "Switches", 256, &(sw_args){SW1_PIN}, tskIDLE_PRIORITY+5, NULL);
    vTaskStartScheduler();

    while(1);
}


void monitor_slave(void* notUsed){
	const uint16_t WAIT = 20;
	uint8_t buffer;
	//Waits for a signal from motor_cycle to indicate the motor has started
	xQueueReceive(SPIReadQueue, &buffer, portMAX_DELAY);
	while(1){
		xQueueSendToBack(SPIQueue, &((spi_send){STAT_REG|READ_COM, 0}), portMAX_DELAY);
		BaseType_t received = xQueueReceive(SPIReadQueue, &buffer, WAIT);
		if(received == pdFALSE || buffer != 1){
			printf("Buffer = %i\n", buffer);
			xQueueSendToBack(LEDQueue, &((led_behavior){0, 1, 0, 10, 0.1, false}), portMAX_DELAY);
		} else xQueueSendToBack(LEDQueue, &((led_behavior){0, 1, 0, 10, 0.9, false}), portMAX_DELAY);
		printf("monitor: %8b\n", buffer);
		vTaskDelay(2000);
	}
}


void ext_LED(void* notUsed){
	led_behavior b = {1, 0, 0, 1, 1, false};
	double onDur=1000, offDur=0;
	while(1){
		if(xQueueReceive(LEDQueue, &b, 0) == pdTRUE){
			float cycle = 1000.0/b.freq;
			onDur = cycle * b.duty;
			offDur = cycle - onDur;
		} 
		gpio_put(RED_LED_PIN, b.R);
		gpio_put(GRN_LED_PIN, b.G);
		gpio_put(BLU_LED_PIN, b.B);
		vTaskDelay(onDur);
		gpio_put(RED_LED_PIN, 0);
		gpio_put(GRN_LED_PIN, 0);
		gpio_put(BLU_LED_PIN, 0);
		vTaskDelay(offDur);

		//This check doesn't make much sense to be here, but it required
		//the least amount of change to accomplish the intended effect
		if(b.doStop && (abs(get_set_position() - motor_get_position()) < 20)){
			xQueueSendToBack(LEDQueue, &((led_behavior){0, 1, 0, 10, 0.9, false}), portMAX_DELAY);
		}
	}
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
	init_helper(RED_LED_PIN, GPIO_OUT, false);
	init_helper(GRN_LED_PIN, GPIO_OUT, false);
	init_helper(BLU_LED_PIN, GPIO_OUT, false);
	// ext_LED(RED_LED_PIN);

	init_helper(SW1_PIN, GPIO_IN, false);
	gpio_pull_up(SW1_PIN);
	init_helper(SW2_PIN, GPIO_IN, false);
	gpio_pull_up(SW2_PIN);

	init_helper(CS_PIN, GPIO_OUT, false);
    init_helper(CLK_PIN, GPIO_OUT, false);
    init_helper(MOSI_PIN, GPIO_OUT, false);
    init_helper(MISO_PIN, GPIO_IN, false);
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


////////////////////
//Motor Functions///

void motor_cycle(void* notUsed){
	uint8_t posInst;
	const uint8_t NUM_POSITIONS = 9;
	int32_t positions[] = {0, 435, 870, 1305, 1740, 3480, 5220, -5220, -3480};
	int32_t setPos = 0;
	bool firstComm = true;
	for(int i = 0;;){
		do{ xQueueReceive(positionQueue, &posInst, portMAX_DELAY); } 
		while(posInst != NEXT_POS);
		xQueueSendToBack(LEDQueue, &((led_behavior){1, 0, 0, 10, 0.5, true}), portMAX_DELAY);
		while(posInst != STOP_POS){
			xQueueReceive(positionQueue, &posInst, portMAX_DELAY);
			switch(posInst){
				case NEXT_POS:
					set_motor_status(M_BUSY);
					xQueueSendToBack(LEDQueue, &((led_behavior){1, 0, 0, 10, 0.5, true}), portMAX_DELAY);
					i = (i+1)%NUM_POSITIONS;
					send_motor_cmd(setPos = positions[i]);
					if(firstComm){ xQueueSendToBack(SPIReadQueue, &posInst, portMAX_DELAY); firstComm = false; }
					break;
				case PREV_POS: 
					set_motor_status(M_BUSY);
					xQueueSendToBack(LEDQueue, &((led_behavior){1, 0, 0, 10, 0.5, true}), portMAX_DELAY);
					i = (i-1+NUM_POSITIONS)%NUM_POSITIONS; //adding NUM_POSITIONS ensures value stays positive
					send_motor_cmd(setPos = positions[i]);
					if(firstComm){ xQueueSendToBack(SPIReadQueue, &posInst, portMAX_DELAY); firstComm = false; }
					break;
				case STOP_POS:
					set_motor_status(M_IDLE);
					xQueueSendToBack(LEDQueue, &((led_behavior){1, 0, 0, 10, 1, false}), portMAX_DELAY);
					send_motor_cmd(setPos = motor_get_position());
					break;
			}
		}
	}
}


////////////////////
//SPI Functions/////

void send_motor_cmd(int32_t pos){
	xQueueSendToBack(SPIQueue, &((spi_send){MOTOR_CMD_0|WRITE_COM, pos&0xFF}), portMAX_DELAY);
	xQueueSendToBack(SPIQueue, &((spi_send){MOTOR_CMD_8|WRITE_COM, (pos>>8)&0xFF}), portMAX_DELAY);
	xQueueSendToBack(SPIQueue, &((spi_send){MOTOR_CMD_16|WRITE_COM, (pos>>16)&0xFF}), portMAX_DELAY);
	xQueueSendToBack(SPIQueue, &((spi_send){MOTOR_CMD_24|WRITE_COM, (pos>>24)&0xFF}), portMAX_DELAY);
	xQueueSendToBack(SPIQueue, &((spi_send){CMD_REG|WRITE_COM, SET_MOTOR_POS}), portMAX_DELAY);
	motor_move(pos);
}


//8 Got no errors with no delay between commands in the master and
//Set to 80 for debugging purposes
#define SLEEP 80
#define send_bit(pin, bit){\
	sleep_us(SLEEP);\
	gpio_put((pin), (bit));\
}


uint8_t read_byte(){
    uint8_t output = 0;
    for(int i = 0; i < 8; i++){
        send_bit(CLK_PIN, 1);
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
	spi_send command;
	//Send pointless command to ensure they're in sync
	send_command(LED_REG|WRITE_COM, 0);
	while(1){
		xQueueReceive(SPIQueue, &command, portMAX_DELAY);
		printf("Byte 1: %8b; Byte 2: %8b\n", command.byte1, command.byte2);
		uint8_t output = send_command(command.byte1, command.byte2);
		if(command.byte1&READ_COM) xQueueSendToBack(SPIReadQueue, &output, portMAX_DELAY);
	}
}


////////////////////
//Interrupts////////

void gpio_int_callback(uint gpio, uint32_t events){
}


bool switch_state_change(sw_state* state, bool sw1Cond, sw_state trans1, bool sw2Cond, sw_state trans2){
	if(gpio_get(SW1_PIN) == sw1Cond){
		*state = trans1;
		return true;
	} 
	if(gpio_get(SW2_PIN) == sw2Cond){
		*state = trans2;
		return true;
	}
	return false;
}


void switch_handler(void* args){
	xQueueSendToBack(positionQueue, &STOP_POS, portMAX_DELAY);
	const uint8_t pollRate = 30;
	sw_state state = SW_NONE;
	bool change = false;

	for(int i = 0;;){
		switch(state){
			case SW_NONE: change = switch_state_change(&state, 0, SW_1, 0, SW_2); break;
			case SW_1:
				if(change) xQueueSendToBack(positionQueue, &NEXT_POS, portMAX_DELAY);
				change = switch_state_change(&state, 1, SW_NONE, 0, SW_BOTH);
				break;
			case SW_2:
				if(change) xQueueSendToBack(positionQueue, &PREV_POS, portMAX_DELAY);
				change = switch_state_change(&state, 0, SW_BOTH, 1, SW_NONE);
				break;
			case SW_BOTH:
				if(change) i = 0;
				else if(i++ >= 500/pollRate) xQueueSendToBack(positionQueue, &STOP_POS, portMAX_DELAY);			
				change = switch_state_change(&state, 1, SW_BETW_1, 1, SW_BETW_2);
				break;
			case SW_BETW_1: change = switch_state_change(&state, 0, SW_BOTH, 1, SW_NONE); break;
			case SW_BETW_2: change = switch_state_change(&state, 1, SW_NONE, 0, SW_BOTH); break;
		}
		vTaskDelay(pollRate);
	}
}




/****************\
| Slave Functions|
\****************/
#else

const uint8_t PHA = 5;
const uint8_t PHB = 4;

#define DEBUG 0
uint8_t test;

static QueueHandle_t instructionQueue;
static QueueHandle_t register_queue;
static QueueHandle_t positionQueue;

int main(){
	stdio_init_all();
    motor_init(0.85, 0, 0, false); //Slave Motor
    hardware_init();

    watchdog_start_tick(12);
	watchdog_enable(200000, false);
	printf(" WatchDog has been initialized\n");

    instructionQueue = xQueueCreate(256, 1);
    register_queue = xQueueCreate(256, sizeof(reg_update));
    positionQueue = xQueueCreate(256, sizeof(int32_t));

    xTaskCreate(heartbeat, "LED_Task",256,NULL,1,NULL);
    xTaskCreate(motor_cycle, "Move Motor",256,NULL,tskIDLE_PRIORITY+3,NULL);
    //This task handles all of the writes to the registers in a single queue
    xTaskCreate(update_registers, "Update Registers",256,NULL,tskIDLE_PRIORITY+4,NULL);
    //Contains the state machine that controlls the SPI communication
    xTaskCreate(slave_state, "Slave State",256,NULL,tskIDLE_PRIORITY+5,NULL);

    vTaskStartScheduler();

    while(1);
}


void motor_cycle(void* notUsed){
	int32_t nextPos;
	while(1){
		xQueueReceive(positionQueue, &nextPos, portMAX_DELAY);
		// printf("BIN: %32b; DEC: %i\n", nextPos, nextPos);
		set_motor_status(M_BUSY);
		motor_move(nextPos);
	}

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

    gpio_set_irq_enabled_with_callback(PHA, GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL, true, &gpio_int_callback);    
    gpio_set_irq_enabled(PHB, GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL, true);    
}


void heartbeat(void * notUsed){
    while(1){
    	watchdog_update();
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
	if(error) {
		printf("A\n");
		return ERR_GET_COMM;
	}

	switch((*byte_1)&0x0F){
		case READ_COM: *state = READ_S; break;
		case WRITE_COM: *state = WRITE_S; break;
		case TESTING_COM: while(1); //For testing watchdog
		default: 
			printf("B\n");
			return ERR_GET_COMM;
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
	static uint8_t registers[12] = {0};
    if(mode == READ_REG) return registers[reg>>4];
    reg_update update = {registers, reg&0xF0, val, mode};
    xQueueSendToBack(register_queue, &update, portMAX_DELAY);
    return 0;
}


void update_registers(void* notUsed){
	reg_update update;
	while(1){
		xQueueReceive(register_queue, &update, portMAX_DELAY);
		uint8_t index = update.reg>>4;
		switch(update.arg){
			case OVERWRITE_REG: update.regArr[index] = update.val; break;
			case OR_REG: update.regArr[index] |= update.val; break;
			case AND_REG: update.regArr[index] &= update.val; break;
			default: break;
		}
		switch(update.reg){
			case CMD_REG: 
				if(update.val&SET_MOTOR_POS){
					int32_t pos = 
						update.regArr[MOTOR_CMD_0>>4] |
						update.regArr[MOTOR_CMD_8>>4]<<8 |
						update.regArr[MOTOR_CMD_16>>4]<<16 |
						update.regArr[MOTOR_CMD_24>>4]<<24;
					xQueueSendToBack(positionQueue, &pos, portMAX_DELAY);
					update.regArr[STAT_REG>>4] = 1;
				}
				if(update.val&GET_MOTOR_POS){
					int32_t currPos = motor_get_position();
					update.regArr[MOTOR_POS_0] = currPos & 0xFF;
					update.regArr[MOTOR_POS_8] = (currPos>>8) & 0xFF;
					update.regArr[MOTOR_POS_16] = (currPos>>16) & 0xFF;
					update.regArr[MOTOR_POS_24] = (currPos>>24) & 0xFF;
				}
				break;
			default: break;
		}
	}
}


////////////////////
//Interrupts////////

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
        case PHA: case PHB: phase_change_irq(gpio, events_unused); break;
    }
}


/****************\
|Shared Functions|
\****************/
#endif