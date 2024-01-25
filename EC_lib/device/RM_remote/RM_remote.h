//=====================================================================================================
// RM_remote.h
//=====================================================================================================
//
//       IRobot  EC_lib
//
// author: @ dji
// modify: @ Specific_Cola
// 
//
//=====================================================================================================
#ifndef RM_REMOTE_H__
#define RM_REMOTE_H__

#include "bsp_usart.h"
#include "struct_typedef.h"

#define RC_FRAME_LENGTH   18u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
#define PRESS                   ((uint8_t)1)
#define RELEASE                 ((uint8_t)0)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)
/* ----------------------- Data Struct ------------------------------------- */
typedef __PACKED_STRUCT
{
        __PACKED_STRUCT
        {
                int16_t ch[5];
                char s[2];
        } rc;
        __PACKED_STRUCT
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
        } mouse;
        __PACKED_STRUCT
        {
                uint16_t v;
        } key;

} RC_ctrl_t;


typedef struct{
    uint8_t statu;
    RC_ctrl_t state_interfaces;
    Usart_Device_t *usart_info;


}RM_Remote_t;

RM_Remote_t *rmRemoteAdd(UART_HandleTypeDef *huart);
void rmRemoteDelete(RM_Remote_t *remote);
void rmRemoteCallback(Usart_Device_t *instance);
uint8_t rmRemoteIsError();
void solveRCLost(void);
void solveDataError(void);
void sbus_to_rc(uint8_t dambufnum);

extern bool_t CheakKeyPress(uint16_t Key);//长按触发
extern bool_t CheakKeyPressOnce(uint16_t Key);//边沿触发
// 归一化摇杆值
extern fp32 RemoteChannalRightX();
extern fp32 RemoteChannalRightY();
extern fp32 RemoteChannalLeftX();
extern fp32 RemoteChannalLeftY();
extern fp32 RemoteDial();

// 归一化鼠标移动
extern fp32 MouseMoveX();
extern fp32 MouseMoveY();

// 鼠标左右键
extern bool_t MousePressLeft();
extern bool_t MousePressRight();

// 拨杆位置检测
extern bool_t SwitchRightUpSide();
extern bool_t SwitchRightMidSide();
extern bool_t SwitchRightDownSide();
extern bool_t SwitchLeftUpSide();
extern bool_t SwitchLeftMidSide();
extern bool_t SwitchLeftDownSide();


extern fp32 NormalizedLimit(fp32 input);
#endif // !RM_REMOTE_H__

