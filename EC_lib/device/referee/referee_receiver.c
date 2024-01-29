#include "referee_receiver.h"
#include "main.h"

static Referee_Receiver_t *referee_receiver_instance;   

Referee_Receiver_t *refereeReceiverAdd(Usart_Device_t *huart)
{
    Referee_Receiver_t *receiver = (Referee_Receiver_t*)malloc(sizeof(Referee_Receiver_t));
    Usart_Register_t usart;
    memset(receiver,0,sizeof(Referee_Receiver_t));
    usart.usart_handle = huart;
    usart.rx_buff_num = 1;
    usart.rx_len = 256;
    usart.usart_device_callback = refereeReceiverCallback;
    receiver->usart_info = usartDeviceRegister(&usart);
	
	referee_receiver_instance = receiver;
    return receiver;

}

void refereeReceiverCallback(Usart_Device_t *usart)
{
    
}