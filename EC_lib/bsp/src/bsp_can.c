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
#include "bsp_can.h"
#include "main.h"

static  Can_Device_t *can_device[CAN_MX_REGISTER_CNT] = {NULL};
static uint8_t id_cnt; // ȫ��CANʵ������,ÿ�����µ�ģ��ע�������

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
        Error_Handler();//����ϣ������һ��ȫ�ֱ�����չʾ��������
    }
    for (uint8_t i = 0;i<id_cnt;i++)
    {
        if (can_device[i]->rx_id == register->rx_id && can_device[i]->can_handle == register->can_handle) 
        {
            Error_Handler();
        }
    }

    Can_Device_t *instance = (Can_Device_t *)malloc(sizeof(Can_Device_t)); // ����ռ�
    memset(instance, 0, sizeof(Can_Device_t));                           // ����Ŀռ�δ����0,����Ҫ�����
    // ���з��ͱ��ĵ�����
    instance->tx_config.StdId = register->tx_id; // ����id
    instance->tx_config.IDE = CAN_ID_STD;      // ʹ�ñ�׼id,��չid��ʹ��CAN_ID_EXT(Ŀǰû������)
    instance->tx_config.RTR = CAN_RTR_DATA;    // ��������֡
    instance->tx_config.DLC = register->tx_dlc;            // Ĭ�Ϸ��ͳ���Ϊ8
    // ���ûص������ͽ��շ���id
    instance->can_handle = register->can_handle;
    instance->tx_id = register->tx_id; // ����û��,����ɾ��
    instance->rx_id = register->rx_id;
    instance->can_device_callback = register->can_device_callback;
    instance->id = register->id;

    canFilterConfig(instance);         // ���CAN����������
    can_device[id_cnt++] = instance; // ��ʵ�����浽can_instance��

    return instance; // ����canʵ��ָ��

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
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0)) // FIFO��Ϊ��,�п����������ж�ʱ�ж�֡���ݽ���
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_config, can_rx_buff); // ��FIFO�л�ȡ����
        for (uint8_t i = 0; i < id_cnt; i++)
        { // �������˵������Ҫ�ҵ�ʵ��
            if (hcan == can_device[i]->can_handle && rx_config.StdId == can_device[i]->rx_id)
            {
                if (can_device[i]->can_device_callback != NULL) // �ص�������Ϊ�վ͵���
                {
                    can_device[i]->rx_len = rx_config.DLC;                      // ������յ������ݳ���
                    memcpy(can_device[i]->rx_buff, can_rx_buff, rx_config.DLC); // ��Ϣ��������Ӧʵ��
                    can_device[i]->can_device_callback(can_device[i]);     // �����ص��������ݽ����ʹ���
                }
                return;
            }
        }
    }


}