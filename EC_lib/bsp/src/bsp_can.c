//=====================================================================================================
// bsp_can.h
//=====================================================================================================
//
//       IRobot  EC_lib
// 仅供交流使用，禁止商用
//最终解释权归西电IRobot电控组
// question:  specificcola@proton.me
// Date			Author			Notes
// 
//
//=====================================================================================================
#include "bsp_can.h"
#include "main.h"

static  Can_Device_t *can_device[CAN_MX_REGISTER_CNT] = {NULL};
static uint8_t id_cnt; // 全局CAN实例索引,每次有新的模块注册会自增

void canFilterConfig(Can_Device_t *device)
{
    CAN_FilterTypeDef can_filter_config;
    can_filter_config.FilterActivation = ENABLE;
    can_filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_config.FilterIdHigh = 0x0000;
    can_filter_config.FilterIdLow = 0x0000;
    can_filter_config.FilterMaskIdHigh = 0x0000;
    can_filter_config.FilterMaskIdLow = 0x0000;
    can_filter_config.FilterBank = 0;
    can_filter_config.FilterFIFOAssignment = CAN_RX_FIFO0;\
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_config);
    
    can_filter_config.SlaveStartFilterBank = 14;
    can_filter_config.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_config);
}

Can_Device_t *canDeviceRegister(Can_Register_t *register)
{
    if(!id_cnt)
    {
        canOnInit();
        canOnActivate();
    }
    if(id_cnt > CAN_MX_REGISTER_CNT)
    {
        Error_Handler();//后面希望定义一个全局变量来展示错误类型
    }
    for (uint8_t i = 0;i<id_cnt;i++)
    {
        if (can_device[i]->rx_id == register->rx_id && can_device[i]->can_handle == register->can_handle) 
        {
            Error_Handler();
        }
    }

    Can_Device_t *instance = (Can_Device_t *)malloc(sizeof(Can_Device_t)); // 分配空间
    memset(instance, 0, sizeof(Can_Device_t));                           // 分配的空间未必是0,所以要先清空
    // 进行发送报文的配置
    instance->tx_config.StdId = register->tx_id; // 发送id
    instance->tx_config.IDE = CAN_ID_STD;      // 使用标准id,扩展id则使用CAN_ID_EXT(目前没有需求)
    instance->tx_config.RTR = CAN_RTR_DATA;    // 发送数据帧
    instance->tx_config.DLC = register->tx_dlc;            // 默认发送长度为8
    // 设置回调函数和接收发送id
    instance->can_handle = register->can_handle;
    instance->tx_id = register->tx_id; // 好像没用,可以删掉
    instance->rx_id = register->rx_id;
    instance->can_device_callback = register->can_device_callback;
    instance->id = register->id;

    canFilterConfig(instance);         // 添加CAN过滤器规则
    can_device[id_cnt++] = instance; // 将实例保存到can_instance中

    return instance; // 返回can实例指针

}

void canOnInit(void)
{
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void canOnActivate(void)
{
    HAL_CAN_Start(&hcan1);
    HAL_CAN_Start(&hcan2);
}

void canOnDeactivate(void)
{
    HAL_CAN_Stop(&hcan1);
    HAL_CAN_Stop(&hcan2);
 
}

void canSendMessage(Can_Device_t *instance, uint8_t *message)
{    
    memcpy(instance->tx_buff, message, sizeof(message));
    HAL_CAN_AddTxMessage(instance->can_handle, &instance->tx_config, message, &instance->tx_mailbox);
}

static void canReceiveMessage(CAN_HandleTypeDef *hcan)
{
    static CAN_RxHeaderTypeDef rx_config;
    static uint8_t can_rx_buff[8];
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0)) // FIFO不为空,有可能在其他中断时有多帧数据进入
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_config, can_rx_buff); // 从FIFO中获取数据
        for (uint8_t i = 0; i < id_cnt; i++)
        { // 两者相等说明这是要找的实例
            if (hcan == can_device[i]->can_handle && rx_config.StdId == can_device[i]->rx_id)
            {
                if (can_device[i]->can_device_callback != NULL) // 回调函数不为空就调用
                {
                    can_device[i]->rx_len = rx_config.DLC;                      // 保存接收到的数据长度
                    memcpy(can_device[i]->rx_buff, can_rx_buff, rx_config.DLC); // 消息拷贝到对应实例
                    can_device[i]->can_device_callback(can_device[i]);     // 触发回调进行数据解析和处理
                }
                return;
            }
        }
    }


}