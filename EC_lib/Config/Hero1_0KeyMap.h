#ifndef _HERO1_0_KEY_MAP_
#define _HERO1_0_KEY_MAP_

#include "Remote.h"
#define YAW_REMOTE_SENS                         0.25f
#define PITCH_REMOTE_SENS                       0.25f
#define YAW_MOUSE_SENS                          100
#define PITCH_MOUSE_SENS                        50

// ��̨�˶�����ָ��
// YAW
#define GIMBAL_CMD_YAW_KEYMAP           NormalizedLimit(MouseMoveY()*YAW_MOUSE_SENS + RemoteChannalLeftY()*YAW_REMOTE_SENS)
// PITCH
#define GIMBAL_CMD_PITCH_KEYMAP         -NormalizedLimit(MouseMoveX()*PITCH_MOUSE_SENS + RemoteChannalLeftX()*PITCH_REMOTE_SENS)

// �����˶�����ָ��
// ǰ��
#define CHASSIS_CMD_X_KEYMAP            NormalizedLimit((RemoteChannalRightX() + CheakKeyPress(KEY_PRESSED_OFFSET_W) - CheakKeyPress(KEY_PRESSED_OFFSET_S)))
// ����
#define CHASSIS_CMD_Y_KEYMAP            NormalizedLimit((RemoteChannalRightY() + CheakKeyPress(KEY_PRESSED_OFFSET_A) - CheakKeyPress(KEY_PRESSED_OFFSET_D)))
// ����
#define CHASSIS_HIGH_SPEED_KEYMAP       CheakKeyPress(KEY_PRESSED_OFFSET_CTRL)
// ����
#define CHASSIS_STOP_KEYMAP             CheakKeyPress(KEY_PRESSED_OFFSET_Z)
// С���ݿ���
#define CHASSIS_ROTATE_SWITCH_KEYMAP    CheakKeyPress(KEY_PRESSED_OFFSET_SHIFT) || (RemoteDial() == -1.0f)



// �����ջ�����
#define HEAT_CLOSED_LOOP_SWITCH_KEYMAP  CheakKeyPressOnce(KEY_PRESSED_OFFSET_B)
// ��Ƶ����
// ������Ƶ
#define SHOOT_FREQ_RAISE_KEYMAP         CheakKeyPressOnce(KEY_PRESSED_OFFSET_E)
// ������Ƶ
#define SHOOT_FREQ_REDUCE_KEYMAP        CheakKeyPressOnce(KEY_PRESSED_OFFSET_Q)



// ״̬������
// ��̨����
//#define GIMBAL_WEAK_KEYMAP              SwitchRightDownSide()
// ��̨ʹ��
#define GIMBAL_ENABLE_KEYMAP            SwitchRightMidSide() || SwitchRightUpSide()
// �������ʹ��
#define SHOOTER_ENABLE_KEYMAP           SwitchRightUpSide()
// ����ʹ��
#define CHASSIS_ENABLE_KEYMAP           SwitchLeftDownSide()


// ����ָ��
#define SHOOT_COMMAND_KEYMAP            ((RemoteDial() == 1.0f) || (MousePressLeft()))
// ����ָ��
#define AIMBOT_COMMAND_KEYMAP           (SwitchLeftUpSide() || MousePressRight())





#endif
