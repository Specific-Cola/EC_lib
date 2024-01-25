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
#include <stdlib.h>
#include <string.h>

#define RC_CHANNAL_ERROR_VALUE  700
static RM_Remote_t *remote_instance;
static uint16_t KeyFormerChannal = 0;
static uint16_t KeyJumpChannal = 0;
static uint16_t KeyUsed = 0;

//取正函数
static int16_t RC_abs(int16_t value);

RM_Remote_t *rmRemoteAdd(UART_HandleTypeDef *huart)
{
    RM_Remote_t *remote = (RM_Remote_t *)malloc(sizeof(RM_Remote_t));
    Usart_Register_t usart;
    memset(remote,0,sizeof(RM_Remote_t));
    usart.usart_handle = huart;
    usart.rx_buff_num = 2;
    usart.rx_len = RC_FRAME_LENGTH;
    usart.usart_device_callback = rmRemoteCallback;
    remote->usart_info = usartDeviceRegister(&usart);
    remote_instance = remote;
    return remote;
}

void rmRemoteCallback(Usart_Device_t *usart)
{
    static uint16_t this_time_rx_len = 0;
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
//        // __HAL_DMA_ENABLE(usart->usart_handle->hdmarx);//回调外面打开了

        if (this_time_rx_len == RC_FRAME_LENGTH)
        {
            //处理遥控器数据
            sbus_to_rc(0);
            //记录数据接收时间
                
        }
    }
    else//使用缓存区2
    {
        /* Current memory buffer used is Memory 1 */
        // disable DMA
        // 失效DMA
        __HAL_DMA_DISABLE(usart->usart_handle->hdmarx);

        // get receive data length, length = set_data_length - remain_length
        // 获取接收数据长度,长度 = 设定长度 - 剩余长度
        this_time_rx_len = USART_RXBUFF_LIMIT / 2 - usart->usart_handle->hdmarx->Instance->NDTR;

        // reset set_data_lenght
        // 重新设定数据长度
        usart->usart_handle->hdmarx->Instance->NDTR = USART_RXBUFF_LIMIT / 2;

        // set memory buffer 0
        // 设定缓冲区0
        DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

        // enable DMA
        // 使能DMA
        //  __HAL_DMA_ENABLE(usart->usart_handle->hdmarx);//外面打开了

        if (this_time_rx_len == RC_FRAME_LENGTH)
        {
            // 处理遥控器数据
            sbus_to_rc(1);
            // 记录数据接收时间
        }
    }
}



//取正函数
static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}

uint8_t rmRemoteIsError()
{
    //禁止使用go to语句！！！！！！
    if (RC_abs(remote_instance->state_interfaces.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
    {
        memset(&remote_instance->state_interfaces, 0, sizeof(RC_ctrl_t));
        return 1;
    }
    if (RC_abs(remote_instance->state_interfaces.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
    {
        memset(&remote_instance->state_interfaces, 0, sizeof(RC_ctrl_t));
        return 1;
    }
    if (RC_abs(remote_instance->state_interfaces.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
    {
        memset(&remote_instance->state_interfaces, 0, sizeof(RC_ctrl_t));
        return 1;
    }
    if (RC_abs(remote_instance->state_interfaces.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
    {
        memset(&remote_instance->state_interfaces, 0, sizeof(RC_ctrl_t));
        return 1;
    }
    if (remote_instance->state_interfaces.rc.s[0] == 0)
    {
        memset(&remote_instance->state_interfaces, 0, sizeof(RC_ctrl_t));
        return 1;
    }
    if (remote_instance->state_interfaces.rc.s[1] == 0)
    {
        memset(&remote_instance->state_interfaces, 0, sizeof(RC_ctrl_t));
        return 1;
    }
    return 0;

}

void solveRCLost()
{
    HAL_UART_ErrorCallback(remote_instance->usart_info->usart_handle);
}
void solveDataError()
{
    HAL_UART_ErrorCallback(remote_instance->usart_info->usart_handle);
}

void sbus_to_rc(uint8_t DmaBufNmb)
{
    KeyFormerChannal = remote_instance->state_interfaces.key.v;
    remote_instance->state_interfaces.rc.ch[0] = (remote_instance->usart_info->rx_buff2[DmaBufNmb][0] | (remote_instance->usart_info->rx_buff2[DmaBufNmb][1] << 8)) & 0x07ff;         //!< Channel 0
    remote_instance->state_interfaces.rc.ch[1] = ((remote_instance->usart_info->rx_buff2[DmaBufNmb][1] >> 3) | (remote_instance->usart_info->rx_buff2[DmaBufNmb][2] << 5)) & 0x07ff;  //!< Channel 1
    remote_instance->state_interfaces.rc.ch[2] = ((remote_instance->usart_info->rx_buff2[DmaBufNmb][2] >> 6) | (remote_instance->usart_info->rx_buff2[DmaBufNmb][3] << 2) |           //!< Channel 2
                         (remote_instance->usart_info->rx_buff2[DmaBufNmb][4] << 10)) &0x07ff;
    remote_instance->state_interfaces.rc.ch[3] = ((remote_instance->usart_info->rx_buff2[DmaBufNmb][4] >> 1) | (remote_instance->usart_info->rx_buff2[DmaBufNmb][5] << 7)) & 0x07ff;  //!< Channel 3
    remote_instance->state_interfaces.rc.s[0] = ((remote_instance->usart_info->rx_buff2[DmaBufNmb][5] >> 4) & 0x0003);                                      //!< Switch left
    remote_instance->state_interfaces.rc.s[1] = ((remote_instance->usart_info->rx_buff2[DmaBufNmb][5] >> 4) & 0x000C) >> 2;                                 //!< Switch right
    remote_instance->state_interfaces.mouse.y = -(remote_instance->usart_info->rx_buff2[DmaBufNmb][6] | (remote_instance->usart_info->rx_buff2[DmaBufNmb][7] << 8));                     //!< Mouse X axis
    remote_instance->state_interfaces.mouse.x = -(remote_instance->usart_info->rx_buff2[DmaBufNmb][8] | (remote_instance->usart_info->rx_buff2[DmaBufNmb][9] << 8));                     //!< Mouse Y axis
    remote_instance->state_interfaces.mouse.z = remote_instance->usart_info->rx_buff2[DmaBufNmb][10] | (remote_instance->usart_info->rx_buff2[DmaBufNmb][11] << 8);                   //!< Mouse Z axis
    remote_instance->state_interfaces.mouse.press_l = remote_instance->usart_info->rx_buff2[DmaBufNmb][12];                                                 //!< Mouse Left Is Press ?
    remote_instance->state_interfaces.mouse.press_r = remote_instance->usart_info->rx_buff2[DmaBufNmb][13];                                                 //!< Mouse Right Is Press ?
    remote_instance->state_interfaces.key.v = remote_instance->usart_info->rx_buff2[DmaBufNmb][14] | (remote_instance->usart_info->rx_buff2[DmaBufNmb][15] << 8);                     //!< KeyBoard value
    remote_instance->state_interfaces.rc.ch[4] = remote_instance->usart_info->rx_buff2[DmaBufNmb][16] | (remote_instance->usart_info->rx_buff2[DmaBufNmb][17] << 8);                  //NULL

    remote_instance->state_interfaces.rc.ch[0] -= RC_CH_VALUE_OFFSET;
    remote_instance->state_interfaces.rc.ch[1] -= RC_CH_VALUE_OFFSET;
    remote_instance->state_interfaces.rc.ch[2] -= RC_CH_VALUE_OFFSET;
    remote_instance->state_interfaces.rc.ch[3] -= RC_CH_VALUE_OFFSET;
    remote_instance->state_interfaces.rc.ch[4] -= RC_CH_VALUE_OFFSET;
    
    
    KeyJumpChannal = (remote_instance->state_interfaces.key.v ^ KeyFormerChannal);

}

bool_t CheakKeyPress(uint16_t Key)
{
    if ((remote_instance->state_interfaces.key.v & Key) == 0)
        return 0;
    
    return 1;
}
bool_t CheakKeyPressOnce(uint16_t Key)
{
    if ((remote_instance->state_interfaces.key.v & Key) == 0) {
        KeyUsed &= (~Key);
        return 0;
    }

    if ((KeyJumpChannal & Key) == 0){
        return 0;
    }
    else{
        if ((KeyUsed & Key) == 0) {
            KeyUsed |= Key;
            return 1;
        } 
        return 0;
    }
}
// 归一化摇杆值
fp32 RemoteChannalRightX()
{
    return (remote_instance->state_interfaces.rc.ch[1] / 660.0f);
}
fp32 RemoteChannalRightY()
{
    return (-remote_instance->state_interfaces.rc.ch[0] / 660.0f);
}
fp32 RemoteChannalLeftX()
{
    return (remote_instance->state_interfaces.rc.ch[3] / 660.0f);
}
fp32 RemoteChannalLeftY()
{
    return (-remote_instance->state_interfaces.rc.ch[2] / 660.0f);
}
fp32 RemoteDial()
{
    return (remote_instance->state_interfaces.rc.ch[4] / 660.0f);
}

// 归一化鼠标移动
fp32 MouseMoveX()
{
    return (remote_instance->state_interfaces.mouse.x / 32768.0f);
}
fp32 MouseMoveY()
{
    return (remote_instance->state_interfaces.mouse.y / 32768.0f);
}

// 鼠标左右键
bool_t MousePressLeft()
{
    return remote_instance->state_interfaces.mouse.press_l;
}
bool_t MousePressRight()
{
    return remote_instance->state_interfaces.mouse.press_r;
}

// 拨杆位置检测
bool_t SwitchRightUpSide()
{
    return (remote_instance->state_interfaces.rc.s[0] == RC_SW_UP);
}
bool_t SwitchRightMidSide()
{
    return (remote_instance->state_interfaces.rc.s[0] == RC_SW_MID);
}
bool_t SwitchRightDownSide()
{
    return (remote_instance->state_interfaces.rc.s[0] == RC_SW_DOWN);
}
bool_t SwitchLeftUpSide()
{
    return (remote_instance->state_interfaces.rc.s[1] == RC_SW_UP);
}
bool_t SwitchLeftMidSide()
{
    return (remote_instance->state_interfaces.rc.s[1] == RC_SW_MID);
}
bool_t SwitchLeftDownSide()
{
    return (remote_instance->state_interfaces.rc.s[1] == RC_SW_DOWN);
}

fp32 NormalizedLimit(fp32 input) {
    if (input > 1.0f) {
        input = 1.0f;
    }
    else if (input < -1.0f) {
        input = -1.0f;
    }
    return input;
}