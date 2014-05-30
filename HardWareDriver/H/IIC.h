#ifndef __STM32_I2C_H
#define __STM32_I2C_H
#include "config.h"
#include "stm32f10x.h"

//驱动接口，GPIO模拟IIC
#define SCL_PIN GPIO_Pin_11
#define SDA_PIN GPIO_Pin_8

#define SCL_H         GPIOA->BSRR = SCL_PIN  
#define SCL_L         GPIOA->BRR  = SCL_PIN  

#define SDA_H         GPIOA->BSRR = SDA_PIN  
#define SDA_L         GPIOA->BRR  = SDA_PIN  

#define SCL_read      GPIOA->IDR  & SCL_PIN 
#define SDA_read      GPIOA->IDR  & SDA_PIN 



/*====================================================================================================*/
/*====================================================================================================*/
bool i2cWriteBuffer(uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data);
bool i2cWrite(uint8_t addr_, uint8_t reg_, uint8_t data);
bool i2cRead(uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf);
void i2cInit(void);
uint16_t i2cGetErrorCounter(void);
static void i2cUnstick(void);

/*====================================================================================================*/
/*====================================================================================================*/
#endif
