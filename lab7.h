#ifndef LAB7_H
#define LAB7_H


const uint8_t LED_PIN = 25;


const uint8_t READ_COM = 0x0a;    //0b 0000 1010
const uint8_t WRITE_COM = 0x0b;   //0b 0000 1011
const uint8_t TESTING_COM = 0x0c; //0b 0000 1100

#define CS_PIN   17  /* GPIO17 */
#define CLK_PIN  18  /* GPIO18 */
#define MOSI_PIN 19  /* GPIO19 */
#define MISO_PIN 16  /* GPIO16 */

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

const uint8_t CSL_INST = 0x10;
const uint8_t CSH_INST = 0x20;
const uint8_t CLKL_INST = 0x30;
const uint8_t CLKH_INST = 0x40;


void hardware_init();
void heartbeat(void * notUsed);
void gpio_int_callback(uint gpio, uint32_t events);


#endif