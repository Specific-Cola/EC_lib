//=====================================================================================================
// bsp_uart.c
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
#include "bsp_usart.h"

#include <string.h>
#include <stdlib.h>

static  Usart_Device_t *usart_device[USART_MX_REGISTER_CNT] = {NULL};
static uint8_t id_cnt=0; // 全局USART实例索引,每次有新的模块注册会自增

static void usartDMARestart(Usart_Device_t *instance)
{
	switch (instance->rx_buff_num)
	{
	case 1:
		HAL_UARTEx_ReceiveToIdle_DMA(instance->usart_handle, instance->rx_buff, instance->rx_len);
        __HAL_DMA_DISABLE_IT(instance->usart_handle->hdmarx, DMA_IT_HT);
		break;
	case 2:
		//enable the DMA transfer for the receiver request
    	//使能DMA串口接收
    	SET_BIT(instance->usart_handle->Instance->CR3, USART_CR3_DMAR);
    	//enalbe idle interrupt
    	//使能空闲中断
    	__HAL_UART_ENABLE_IT(instance->usart_handle, UART_IT_IDLE);
		//打开回调函数
		instance->usart_handle->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
		instance->usart_handle->RxXferSize = USART_RXBUFF_LIMIT/2;
		__HAL_DMA_ENABLE(instance->usart_handle->hdmarx);
		break;
	default:
		break;
	}
}





static void usartStartReceive(Usart_Device_t* instance){
	if(instance==NULL){
		return ;
	}
	
	if(instance->rx_buff_num==1){
		//单缓冲区
		HAL_UARTEx_ReceiveToIdle_DMA(instance->usart_handle, instance->rx_buff, instance->rx_len);
        __HAL_DMA_DISABLE_IT(instance->usart_handle->hdmarx, DMA_IT_HT);
	}
	else if(instance->rx_buff_num==2){
		//enable the DMA transfer for the receiver request
    	//使能DMA串口接收
    	SET_BIT(instance->usart_handle->Instance->CR3, USART_CR3_DMAR);

    	//enalbe idle interrupt
    	//使能空闲中断
    	__HAL_UART_ENABLE_IT(instance->usart_handle, UART_IT_IDLE);

    	//disable DMA
    	//失效DMA          
    	__HAL_DMA_DISABLE(instance->usart_handle->hdmarx);
    while(instance->usart_handle->hdmarx->Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(instance->usart_handle->hdmarx);
    }

		//双缓冲区
		instance->usart_handle->hdmarx->Instance->PAR = (uint32_t) & (USART3->DR);
		//memory buffer 1
		//内存缓冲区1
		instance->usart_handle->hdmarx->Instance->M0AR = (uint32_t)(instance->rx_buff2[0]);
		//memory buffer 2
		//内存缓冲区2
		instance->usart_handle->hdmarx->Instance->M1AR = (uint32_t)(instance->rx_buff2[1]);
		//data length
		//数据长度
		instance->usart_handle->hdmarx->Instance->NDTR = USART_RXBUFF_LIMIT/2;
		//enable double memory buffer
		//使能双缓冲区
		SET_BIT(instance->usart_handle->hdmarx->Instance->CR, DMA_SxCR_DBM);
		//enable DMA
    	//使能DMA
		instance->usart_handle->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
		instance->usart_handle->RxXferSize = USART_RXBUFF_LIMIT/2;
    	__HAL_DMA_ENABLE(instance->usart_handle->hdmarx);
		
	}
	else{
		Error_Handler();
	}

    
}

Usart_Device_t *usartDeviceRegister(Usart_Register_t *reg){
	
    if(id_cnt > USART_MX_REGISTER_CNT)
    {
        Error_Handler();//后面希望定义一个全局变量来展示错误类型
    }
	
	for (uint8_t i = 0;i<id_cnt;i++)
    {
        if (usart_device[i]->usart_handle == reg->usart_handle) 
        {
            Error_Handler();
        }
    }
	
	Usart_Device_t *instance = (Usart_Device_t *)malloc(sizeof(Usart_Device_t));
	
	if(reg->rx_buff_num > 2){
		reg->rx_buff_num=2;
	}
	if(reg->rx_buff_num < 1){
		reg->rx_buff_num=1;
	}
	
	instance->usart_handle			= reg->usart_handle;
	instance->rx_len 				= reg->rx_len;
	instance->rx_buff_num 			= reg->rx_buff_num;
	instance->usart_device_callback	= reg->usart_device_callback;
	
	usart_device[id_cnt++]=instance;//完成自增
	
	usartStartReceive(instance);//注册完成后直接开启接收
	
	return instance;
}


void usartOnDeactivate(void);

Return_t usartSendMessage(Usart_Device_t *instance, uint8_t *message, uint16_t tx_len,Usart_Transfer_Mode mode){
	
	Return_t ret;
	
	switch(mode){
		case USART_TRANSFER_DMA:
			if(HAL_UART_Transmit_DMA(instance->usart_handle,message,tx_len)!=HAL_OK){
				ret = RETURN_ERROR;
			}
			break;
		
		case USART_TRANSFER_IT:
			if(HAL_UART_Transmit_IT(instance->usart_handle,message,tx_len)!=HAL_OK){
				ret = RETURN_ERROR;
			}
			break;
		
		case USART_TRANSFER_BLOCKING:
			if(HAL_UART_Transmit(instance->usart_handle,message,tx_len,0xFF)!=HAL_OK){
				ret = RETURN_ERROR;
			}
			break;
		
		case USART_TRANSFER_NONE:
			break;
		default:
			break;
	}
	return ret;
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) //todo  双缓存区如何接收
{
    for (uint8_t i = 0; i < id_cnt; ++i)
    { // find the instance which is being                                 handled
        if (huart == usart_device[i]->usart_handle)
        { // call the callback function if it is not NULL
            if (usart_device[i]->usart_device_callback != NULL)
            {
                usart_device[i]->usart_device_callback(usart_device[i]);
                memset(usart_device[i]->rx_buff, 0, size); // 接收结束后清空buffer,对于变长数据是必要的   
				//如果需要清除，就在回调函数里清除
            }
            usartDMARestart(usart_device[i]);
            return; // break the loop
        }
    }
}

uint64_t PE_cnt=0;
uint64_t FE_cnt=0;

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < id_cnt; ++i)
    {
        if (huart == usart_device[i]->usart_handle)
        {
            usartDMARestart(usart_device[i]);
            return;
        }
    }
}
//定义几种串口常用的工作模式，进行配置，例如串口空闲中断







