//=====================================================================================================
// djiMotor.h
//=====================================================================================================
//
//       IRobot  EC_lib
//
// author: @ dji
// modify: @ Specific_Cola
// 
// 
//
//=====================================================================================================
#ifndef DJIMOTOR_H__
#define DJIMOTOR_H__

#include "struct_typedef.h"

typedef enum
{
    MOTOR_6020 = 0,
	MOTOR_3508,
	MOTOR_2006,
	
} Motor_type;

typedef struct{
    uint16_t ecd;
    int32_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    uint16_t last_ecd;
    fp32 angle;

}Motor_Info_t;

typedef struct{
	uint8_t statu;  //online 0  / offline 1 
	uint8_t motor_type; //6020   3508   2006   need add pls contact lwt
    Motor_Info_t *motor_info;
    CAN_HandleTypeDef *hcan;
    uint16_t motor_tx_id;  //0x0000
	uint16_t motor_rx_id;
}DJI_Motor_t;

void djiMotorAdd(DJI_Motor_t *motor);
void djiMotorDelete(DJI_Motor_t *motor);
void djiMotorInfoUpdate(DJI_Motor_t *motor);
void djiMotorSpeedControl(DJI_Motor_t *motor);
void djiMotorPositionControl(DJI_Motor_t *motor);
Return_t djiMotorSendMessage(DJI_Motor_t *motor);
#endif // !DJIMOTOR_H__
