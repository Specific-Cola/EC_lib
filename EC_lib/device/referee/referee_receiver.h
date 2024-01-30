#ifndef REFEREE_RECEIVER_H__
#define REFEREE_RECEIVER_H__

#include "bsp_usart.h"




typedef struct{



}Referee_info_t;

typedef struct{
    uint8_t statu;
    Usart_Device_t *usart_info;
    Referee_info_t state_interfaces;


}Referee_Receiver_t;

Referee_Receiver_t *refereeReceiverAdd(UART_HandleTypeDef *huart);
void refereeReceiverCallback(Usart_Device_t *usart);


#endif // !REFEREE_RECEIVER_H__
