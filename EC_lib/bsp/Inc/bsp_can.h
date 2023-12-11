//=====================================================================================================
// bsp_can.h
//=====================================================================================================
//
//       IRobot  EC_lib
// ��������ʹ�ã���ֹ����
//���ս���Ȩ������IRobot�����
// question:  specificcola@proton.me
// Date			Author			Notes
// 
//
//=====================================================================================================
#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "struct_typedef.h"
#include "main.h"

#define CAN_MX_REGISTER_CNT 16

typedef struct _ {

    CAN_HandleTypeDef *can_handle; // can���
    CAN_TxHeaderTypeDef tx_config;    // CAN���ķ�������
    uint32_t tx_id;                // ����id
    uint32_t tx_mailbox;           // CAN��Ϣ����������
    uint8_t tx_buff[8];            // ���ͻ���
    uint32_t rx_id;                // ����id
    uint8_t rx_buff[8];            // ���ջ���,�����Ϣ����Ϊ8
    uint8_t rx_len;                // ���ճ���,����Ϊ0-8
    // ���յĻص�����,���ڽ������յ�������
    void (*can_device_callback)(struct _ *); // callback needs an instance to tell among registered ones
    void *id;

}Can_Device_t;

typedef struct{
    CAN_HandleTypeDef *can_handle;              // can���
    uint32_t tx_id;                             // ����id
    uint32_t rx_id;                              // ����id
    uint8_t tx_dlc;                         
    void (*can_device_callback)(Can_Device_t *); // ����������ݵĻص�����
    void *id;    
}Can_Register_t;



extern void canFilterConfig(void);
Can_Device_t *canDeviceRegister(Can_Register_t *register)
extern void canOnInit(void);
extern void canOnActivate(void);
extern void canOnDeactivate(void);
extern void canSendMessage(CAN_HandleTypeDef *hcan, uint32_t id, uint32_t dlc, uint8_t *message);
extern void canReceiveMessage(void);

#endif // BSP_CAN_H
