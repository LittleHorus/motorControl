#include "stdint.h"

uint32_t i = 0;
uint8_t driverSpeed = 0;
uint8_t microStop = 0;


//USART defines
/******************************************************************************/
#define DE_RX_MODE GPIOA->BSRR = GPIO_Pin_15
#define DE_TX_MODE GPIOA->BRR = GPIO_Pin_15

/******************************************************************************/

//MODBUS define
/******************************************************************************/
#define MODBUS_INIT     0
#define MODBUS_START    1
#define MODBUS_STOP     2
#define MODBUS_SETFR    3
#define MODBUS_READFR   4
#define MODBUS_REVERSE  5
#define MODBUS_RESET    6
#define MODBUS_START_SETFR 7
#define MODBUS_REVERSE_SETFR 8
#define MODBUS_BLOCKREAD 9
/******************************************************************************/

//DRIVER
/******************************************************************************/
#define BRAKE_ON GPIO_ResetBits(GPIOB, GPIO_Pin_15)
#define BRAKE_OFF GPIO_SetBits(GPIOB, GPIO_Pin_15)
#define DRIVER_ENABLE GPIO_SetBits(GPIOB, GPIO_Pin_7)
#define DRIVER_DISABLE GPIO_ResetBits(GPIOB, GPIO_Pin_7)
#define DRIVER_DIR_FORWARD GPIO_ResetBits(GPIOA, GPIO_Pin_10)
#define DRIVER_DIR_REVERSE GPIO_SetBits(GPIOA, GPIO_Pin_10)

#define SET_LOW_RST_PIN GPIO_ResetBits(GPIOB, GPIO_Pin_7)
#define SET_HIGH_RST_PIN GPIO_SetBits(GPIOB, GPIO_Pin_7)

#define SPI_SCLK_LOW GPIO_ResetBits(GPIOB, GPIO_Pin_3)
#define SPI_SCLK_HIGH GPIO_SetBits(GPIOB, GPIO_Pin_3)
#define SPI_MOSI_LOW GPIO_ResetBits(GPIOB, GPIO_Pin_5)
#define SPI_MOSI_HIGH GPIO_SetBits(GPIOB, GPIO_Pin_5)
#define SPI_MISO GPIO_ReadInputDataBits(GPIOB, GPIO_Pin_4)

#define SPI1_CS_LOW GPIO_ResetBits(GPIOB, GPIO_Pin_6)
#define SPI1_CS_HIGH GPIO_SetBits(GPIOB, GPIO_Pin_6)

#define HALL_POWER_ON GPIOB->BSRR = GPIO_Pin_11
#define HALL_POWER_OFF GPIOB->BRR = GPIO_Pin_11

#define BRAKE_RESISTOR_ENABLE GPIOB->BRR = GPIO_Pin_13
#define BRAKE_RESISTOR_DISABLE GPIOB->BSRR = GPIO_Pin_13
/******************************************************************************/

//NET_ADRESS
/******************************************************************************/
#define NET_ADRESS_BIT_0 (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13))
#define NET_ADRESS_BIT_1 (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14))
#define NET_ADRESS_BIT_2 (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15))


//uint8_t NetAdress = 0;
/******************************************************************************/

uint16_t ADC_Value = 0;
uint8_t driverStatus = 0;
#define ADC__THRESHOLD 2729//700 from half range



//BUTTON
#define MCU_BUTTON GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0)
uint16_t iButton = 0;
uint16_t buttonPress = 0;
uint8_t stateButton = 0, preStateButton = 0;
uint8_t buttonPressCounts = 0;
#define TIMEOUT_BUTTON 100
#define BUTTON_THRESHOLD 60
uint16_t faultI = 0, timeout = 0;


#define CW_STATUS GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12)
#define CCW_STATUS GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11)
#define FAULT__STATUS GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)
#define SpeedEnableButtonState GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10)

#define HALL_A_PIN GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0)
#define HALL_B_PIN GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1)
#define HALL_C_PIN GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_2)

uint8_t led_status = 0;
#define LED__STOP 2
#define LED__RUN 4
#define LED__BRAKE 6


uint8_t braking = 0, brakingOn = 0, brake_flag=0, fwd_flag=0 /*brake_ended=0*/; //переменная для задания шагов уменьшения скорости при торможении
uint16_t led_i = 0, lowLevelLed = 5000, fwd_counter=0, bkwd_counter=0;

uint16_t time_to_stop = 0;

uint16_t sw_count = 0;
uint16_t sc = 0;
int8_t FLAG__ = 0;
int8_t STATE = 0;
int8_t STATE_N = 0;
volatile uint8_t LED_CHECK = 0;


const uint8_t crc8Table[256] = {
0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83, 0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41, 0x9D, 0xC3, 0x21, 0x7F,
0xFC, 0xA2, 0x40, 0x1E, 0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC, 0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0,
0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62, 0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D, 0x7C, 0x22, 0xC0, 0x9E,
0x1D, 0x43, 0xA1, 0xFF, 0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5, 0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58, 0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A, 0x65, 0x3B, 0xD9, 0x87,
0x04, 0x5A, 0xB8, 0xE6, 0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24, 0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B,
0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9, 0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F, 0x4E, 0x10, 0xF2, 0xAC,
0x2F, 0x71, 0x93, 0xCD, 0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92, 0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C, 0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE, 0x32, 0x6C, 0x8E, 0xD0,
0x53, 0x0D, 0xEF, 0xB1, 0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73, 0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49,
0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B, 0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4, 0x95, 0xCB, 0x29, 0x77,
0xF4, 0xAA, 0x48, 0x16, 0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A, 0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7, 0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35};
