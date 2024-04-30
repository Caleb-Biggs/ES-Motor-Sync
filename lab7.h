#ifndef LAB7_H
#define LAB7_H

//TODO: organize this

//Commands
const uint8_t READ_COM = 0x0a;    //0b 0000 1010
const uint8_t WRITE_COM = 0x0b;   //0b 0000 1011
const uint8_t TESTING_COM = 0x0c; //0b 0000 1100


//Registers
const uint8_t LED_REG = 0x10; //0b 0001 0000
    //LED_REG Data
    const uint8_t ILL_BLUE = 1;      //Illuminates the green LED
    const uint8_t ILL_RED = 1<<1;    //Illuminates the green LED
    const uint8_t ILL_GREEN = 1<<2;  //Illuminates the green LED
const uint8_t SW_REG = 0x20;  //0b 0010 0000
    //SW_REG Data
    const uint8_t SW1_STATE = 1;     //Current state of switch 1
    const uint8_t SW2_STATE = 1<<1;  //Current state of switch 2
    const uint8_t SW1_INTER = 1<<2;  //1 if switch 1 caused an interrupt
    const uint8_t SW2_INTER = 1<<3;  //1 if switch 2 caused an interrupt
const uint8_t INT_REG = 0x30; //0b 0011 0000
    //INT_REG Data
    const uint8_t ACK_SW1_INT = 1;   //Acknowledge switch 1 interrupt
    const uint8_t ACK_SW2_INT = 1<<1;//Acknowledge switch 2 interrupt
    const uint8_t EN_SW1_INT = 1<<2; //Enable switch 1 interrupt
    const uint8_t EN_SW2_INT = 1<<3; //Enable switch 2 interrupt

const uint8_t MOTOR_POS_0 = 0;
const uint8_t MOTOR_POS_8 = 1;
const uint8_t MOTOR_POS_16 = 2;
const uint8_t MOTOR_POS_24 = 3;

const uint8_t MOTOR_CMD_0 = 4;
const uint8_t MOTOR_CMD_8 = 5;
const uint8_t MOTOR_CMD_16 = 6;
const uint8_t MOTOR_CMD_24 = 7;

const uint8_t CMD_REG = 10;
const uint8_t STAT_REG = 11;


//Pins
const uint8_t LED_PIN = 25;
const uint8_t SW1_PIN = 15;
const uint8_t SW2_PIN = 14;

const uint8_t MOSI_PIN = 19;
const uint8_t CLK_PIN = 18;
const uint8_t CS_PIN = 17;	
const uint8_t MISO_PIN = 16;


//Slave Error Codes
const int8_t ERR_GET_COMM = -1;
const int8_t ERR_WRT_DATA = -2;
const int8_t ERR_REG_VAL = -3;
const int8_t ERR_COMM_EX = -4;
const int8_t ERR_SHLDNT_RCH = -5;
const int8_t ERR_SEND_REG = -6;
const int8_t ERR_EXP_CS_LO = -7;
const int8_t ERR_EXP_CS_HI = -8;
const int8_t ERR_EXP_CLK_LO = -9;
const int8_t ERR_EXP_CLK_HI = -10;
const int8_t ERR_GET_BYTE = -11;
const int8_t ERR_REGISTER = -12;


//Slave SPI Instructions
const uint8_t CSL_INST = 0x10;
const uint8_t CSH_INST = 0x20;
const uint8_t CLKL_INST = 0x30;
const uint8_t CLKH_INST = 0x40;


typedef enum STATE {IDLE_S, COMMAND_S, READ_S, WRITE_S, DONE_S} STATE;
typedef enum REG_ARG { READ_REG, OVERWRITE_REG, AND_REG, OR_REG } REG_ARG;
typedef struct REG_UPDATE { uint8_t* reg; uint8_t val; REG_ARG arg; } reg_update;


//Shared Functions
void hardware_init();
void heartbeat(void * notUsed);
void gpio_int_callback(uint gpio, uint32_t events);

//Master Functions
void master_spi(void* notUsed);
void motor_cycle(void* notUsed);

//Slave Functions
void slave_state(void* notUsed);
void update_registers(void* notUsed);
uint8_t registers(REG_ARG mode, uint8_t reg, uint8_t val);

#endif