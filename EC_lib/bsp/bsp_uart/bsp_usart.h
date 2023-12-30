//=====================================================================================================
// bsp_uart.h
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
#ifndef BSP_USART_H
#define BSP_USART_H

#include "struct_typedef.h"
#include "main.h"

#define USART_MX_REGISTER_CNT 3
#define USART_RXBUFF_LIMIT 256 // 如果协议需要更大的buff,请修改这里

/* 发送模式枚举 */
typedef enum
{
    USART_TRANSFER_NONE=0,
    USART_TRANSFER_BLOCKING,
    USART_TRANSFER_IT,
    USART_TRANSFER_DMA,
} Usart_Transfer_Mode;

typedef struct Usart_Device_{

	UART_HandleTypeDef *usart_handle;				// uart句柄
	union {
		uint8_t rx_buff[USART_RXBUFF_LIMIT];		//单缓冲区使用
		uint8_t rx_buff2[2][USART_RXBUFF_LIMIT/2];	//双缓存区使用
	};
	uint8_t rx_len;									//接收一包数据的大小
	uint8_t rx_buff_num;							//缓存区数目
    // 接收的回调函数,用于解析接收到的数据
    void (*usart_device_callback)(struct Usart_Device_ *); // callback needs an instance to tell among registered ones
	
}Usart_Device_t;


typedef struct{
	
	UART_HandleTypeDef *usart_handle;				// uart句柄
	
	uint8_t rx_len;									//接收一包数据的大小
	uint8_t rx_buff_num;							//缓存区数目
	//接收回调函数
    void (*usart_device_callback)(Usart_Device_t *); 
	
}Usart_Register_t;


extern Usart_Device_t *usartDeviceRegister(Usart_Register_t *reg);
extern Return_t usartSendMessage(Usart_Device_t *instance, uint8_t *message, uint16_t tx_len,Usart_Transfer_Mode mode);


#endif