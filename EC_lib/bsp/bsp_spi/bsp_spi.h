//=====================================================================================================
// bsp_spi.h
//=====================================================================================================
//
//       IRobot  EC_lib
//
//GitHub: https://github.com/Specific_Cola
// question:  specificcola@proton.me
// Date			Author			Notes
// 
//
//=====================================================================================================
#ifndef BSP_SPI_H__
#define BSP_SPI_H__

#include "struct_typedef.h"
#include "spi.h"
#include "gpio.h"

#define SPI_DEVICE_CNT 2       // C型开发板引出两路spi,分别连接BMI088/作为扩展IO在8pin牛角座引出
#define MX_SPI_BUS_SLAVE_CNT 4 // 单个spi总线上挂载的从机数目

/* spi transmit recv mode enumerate*/
typedef enum
{
    SPI_BLOCK_MODE = 0, // 默认使用阻塞模式
    SPI_IT_MODE,
    SPI_DMA_MODE,
} SPI_TXRX_MODE_e;

/* SPI实例结构体定义 */
typedef struct SPI_Device_
{
    SPI_HandleTypeDef *spi_handle; // SPI外设handle
    GPIO_TypeDef *GPIOx;           // 片选信号对应的GPIO,如GPIOA,GPIOB等等
    uint16_t cs_pin;               // 片选信号对应的引脚号,GPIO_PIN_1,GPIO_PIN_2等等

    SPI_TXRX_MODE_e spi_work_mode; // 传输工作模式
    uint8_t rx_size;               // 本次接收的数据长度
    uint8_t *rx_buffer;            // 本次接收的数据缓冲区

    void (*callback)(struct SPI_Device_ *); // 接收回调函数
    void *id;                                // 模块指针
} SPI_Device_t;


typedef struct
{
    SPI_HandleTypeDef *spi_handle; // SPI外设handle
    GPIO_TypeDef *GPIOx;           // 片选信号对应的GPIO,如GPIOA,GPIOB等等
    uint16_t cs_pin;               // 片选信号对应的引脚号,GPIO_PIN_1,GPIO_PIN_2等等

    SPI_TXRX_MODE_e spi_work_mode; // 传输工作模式

    void (*callback)(SPI_Device_t *); // 接收回调函数
    void *id;                 // 模块指针
} SPI_Register_t;


SPI_Device_t *spiRegister(SPI_Register_t *conf);



void spiTransmit(SPI_Device_t *spi_ins, uint8_t *ptr_data, uint8_t len);


void spiReceive(SPI_Device_t *spi_ins, uint8_t *ptr_data, uint8_t len);


void spiTransRecv(SPI_Device_t *spi_ins, uint8_t *ptr_data_rx, uint8_t *ptr_data_tx, uint8_t len);


void spiSetMode(SPI_Device_t *spi_ins, SPI_TXRX_MODE_e spi_mode);

#endif