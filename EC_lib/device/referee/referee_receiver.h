#ifndef REFEREE_RECEIVER_H__
#define REFEREE_RECEIVER_H__

#include "bsp_usart.h"
#include "referee_def.h"



typedef struct{
    frame_header_t frame_header;
    uint16_t cmd_id;
    union data
    {
        uint8_t raw_data[119];
        
        
    };
    
    uint16_t frame_tail;




}Referee_info_t;

typedef struct{
    uint8_t statu;
    Usart_Device_t *usart_info;
    Referee_info_t state_interfaces;


}Referee_Receiver_t;

Referee_Receiver_t *refereeReceiverAdd(UART_HandleTypeDef *huart);
void refereeReceiverCallback(Usart_Device_t *usart);


#endif // !REFEREE_RECEIVER_H__
