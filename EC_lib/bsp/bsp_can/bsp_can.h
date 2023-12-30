//=====================================================================================================
// bsp_can.h
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
#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "struct_typedef.h"
#include "can.h"
#include "main.h"

#define CAN_MX_REGISTER_CNT 16

typedef struct Can_Device_ {

    CAN_HandleTypeDef *can_handle; // can句柄
    CAN_TxHeaderTypeDef tx_config;    // CAN报文发送配置
    uint32_t tx_id;                // 发送id
    uint32_t tx_mailbox;           // CAN消息填入的邮箱号
    uint8_t tx_buff[8];            // 发送缓存
    uint32_t rx_id;                // 接收id
    uint8_t rx_buff[8];            // 接收缓存,最大消息长度为8
    uint8_t rx_len;                // 接收长度,可能为0-8
    // 接收的回调函数,用于解析接收到的数据
    void (*can_device_callback)(struct Can_Device_ *); // callback needs an instance to tell among registered ones
    void *id;

}Can_Device_t;

typedef struct{
    CAN_HandleTypeDef *can_handle;              // can句柄
    uint32_t tx_id;                             // 发送id
    uint32_t rx_id;                              // 接收id
    uint8_t tx_dlc;                         
    void (*can_device_callback)(Can_Device_t *); // 处理接收数据的回调函数
    void *id;    
}Can_Register_t;



extern void canFilterConfig(Can_Device_t *device);
Can_Device_t *canDeviceRegister(Can_Register_t *reg);
extern void canOnInit(void);
extern void canOnActivate(void);
extern void canOnDeactivate(void);
extern void canSendMessage(Can_Device_t *instance, uint8_t *message);

#endif // BSP_CAN_H