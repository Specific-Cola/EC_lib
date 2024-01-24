//=====================================================================================================
// RM_remote.c
//=====================================================================================================
//
//       IRobot  EC_lib
//
// author: @ dji
// modify: @ Specific_Cola
// 
//
//=====================================================================================================
#include "RM_remote.h"
#include "bsp_usart.h"
#include "main.h"



RM_Remote_t *rmRemoteAdd(UART_HandleTypeDef *huart)
{
    RM_Remote_t *remote = (RM_Remote_t *)malloc(sizeof(RM_Remote_t));
    Usart_Register_t usart;
    memset(remote,0,sizeof(RM_Remote_t));
    usart.usart_handle = huart;
    usart.rx_buff_num = 1;
    usart.rx_len = RC_FRAME_LENGTH;
    usart.usart_device_callback = rmRemoteCallback;
    remote->usart_info = usartDeviceRegister(&usart);
    return remote;
}

void rmRemoteCallback(Usart_Device_t *usart)
{
    if (usart->usart_handle->Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(usart->usart_handle);
    }
    else if (USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(usart->usart_handle);

        if ((usart->usart_handle->hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(usart->usart_handle->hdmarx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = USART_RXBUFF_LIMIT/2 - usart->usart_handle->hdmarx->Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            usart->usart_handle->hdmarx->Instance->NDTR = USART_RXBUFF_LIMIT/2;

            //set memory buffer 1
            //设定缓冲区1
            usart->usart_handle->hdmarx->Instance->CR |= DMA_SxCR_CT;
            
            // //enable DMA
            // //使能DMA
            // __HAL_DMA_ENABLE(usart->usart_handle->hdmarx);//回调外面打开了

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                sbus_to_rc(0);
                //记录数据接收时间
                
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(usart->usart_handle->hdmarx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = USART_RXBUFF_LIMIT/2 - usart->usart_handle->hdmarx->Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            usart->usart_handle->hdmarx->Instance->NDTR = USART_RXBUFF_LIMIT/2;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            // __HAL_DMA_ENABLE(usart->usart_handle->hdmarx);//外面打开了

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                sbus_to_rc(1);
                //记录数据接收时间
                
            }
        }
    }


}
