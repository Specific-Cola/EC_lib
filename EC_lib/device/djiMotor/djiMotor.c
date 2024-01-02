//=====================================================================================================
// djiMotor.c
//=====================================================================================================
//
//       IRobot  EC_lib
//
// author: @ dji
// modify: @ Specific_Cola
// 
//
//=====================================================================================================
#include "djiMotor.h"
#include "main.h"
#include "bsp_can.h"

static DJI_Motor_t *dji_motor[MAX_DJI_MOTOR_NUM];

DJI_Motor_t *djiMotorAdd(DJI_Motor_t *motor)
{



    return motor;   
}

void djiMotorInfoUpdate(DJI_Motor_t *motor,uint8_t *data)
{
    for (uint8_t i = 0; i < MAX_DJI_MOTOR_NUM; i++) 
    {
        if (dji_motor[i] != NULL)
        {
            motor->motor_info->last_ecd = motor->motor_info->ecd;
            motor->motor_info->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);
            motor->motor_info->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);
            motor->motor_info->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);
            motor->motor_info->temperate = (data)[6];
        }   
        else
            return;
    }
}

