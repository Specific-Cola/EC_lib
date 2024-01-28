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
#include <stdlib.h>
#include <string.h>
#include "user_lib.h"

static DJI_Motor_t *dji_motor[MAX_DJI_MOTOR_NUM];
static uint8_t id_cnt; //记录大疆电机数量

static uint8_t   MotorSendBuffer_can1[24];
static uint8_t   MotorSendBuffer_can2[24];

static uint32_t             send_mail_box_can1;
static uint32_t             send_mail_box_can2;

static void djiMotorCallback(Can_Device_t *instance)
{
    for(uint8_t i = 0; i < id_cnt;i++)
    {
        if(instance == dji_motor[i]->can_info)
        {
            djiMotorInfoUpdate(dji_motor[i],instance->rx_buff);
			
			if(dji_motor[i]->motorCallback!=NULL){
				dji_motor[i]->motorCallback(dji_motor[i]);
			}
			
            break;
        }
    }     
}

DJI_Motor_t *djiMotorAdd(DJI_Motor_Register_t *reg)//使用can instance注册电机    //id就直接填写灯电调闪烁的次数

{
    if (id_cnt > MAX_DJI_MOTOR_NUM) 
    {
        Error_Handler();//电机太多了
    }
    
    Can_Register_t can_reg;
    DJI_Motor_t *motor = (DJI_Motor_t *)malloc(sizeof(DJI_Motor_t));
    memset(&can_reg, 0, sizeof(Can_Register_t));
    memset(motor, 0, sizeof(DJI_Motor_t));
	
    can_reg.can_handle = reg->hcan;
    can_reg.tx_dlc = 8; 
    can_reg.can_device_callback = djiMotorCallback;
	
    switch (reg->motor_type)
    {
    case DJI_MOTOR_6020:
        if (reg->id <= 4)
            can_reg.tx_id = 0x1FF;
        else if (reg->id > 4)
            can_reg.tx_id = 0x2FF;
        can_reg.rx_id = 0x204 + reg->id;
        break;
    case DJI_MOTOR_3508:
        if (reg->id <= 4)
            can_reg.tx_id = 0x200;
        else if (reg->id > 4)
            can_reg.tx_id = 0x1FF;
        can_reg.rx_id = 0x200 + reg->id;
        break;
    case DJI_MOTOR_2006:
        if (reg->id <= 4)
            can_reg.tx_id = 0x200;
        else if (reg->id > 4)
            can_reg.tx_id = 0x1FF;
        can_reg.rx_id = 0x200 + reg->id;
        break;
    default:
        Error_Handler();//电机类型不存在
        break;
    }
    
    motor->motor_type = reg->motor_type;
    motor->can_info = canDeviceRegister(&can_reg);
    dji_motor[id_cnt++] = motor;

    return motor;   
}

Return_t djiMotorSendMessage()
{   
	
	int16_t can_send_num[2][3]={{-1,-1,-1},{-1,-1,-1}};
	Return_t ret=RETURN_SUCCESS;
	
    for(uint8_t i =0;i<id_cnt;i++)
    {
        if(dji_motor[i]->statu == OFFLINE)
        {
            dji_motor[i]->command_interfaces.command = 0;
			ret=RETURN_ERROR;
			continue;
        }


        if(dji_motor[i]->can_info->can_handle == &hcan1)
        {
            MotorSendBuffer_can1[(dji_motor[i]->can_info->rx_id - 0x201)*2 ] = dji_motor[i]->command_interfaces.command >> 8;
            MotorSendBuffer_can1[(dji_motor[i]->can_info->rx_id - 0x201)*2 + 1] = dji_motor[i]->command_interfaces.command;
			
			switch(dji_motor[i]->can_info->tx_id){
				case 0x200:
					can_send_num[0][0]=i;
				break;
				case 0x1FF:
					can_send_num[0][1]=i;
				break;
				case 0x2FF:
					can_send_num[0][2]=i;
				break;
			}
        }
        else if (dji_motor[i]->can_info->can_handle == &hcan2)
        {
            MotorSendBuffer_can2[(dji_motor[i]->can_info->rx_id - 0x201)*2 ] = dji_motor[i]->command_interfaces.command >> 8;
            MotorSendBuffer_can2[(dji_motor[i]->can_info->rx_id - 0x201)*2 + 1] = dji_motor[i]->command_interfaces.command;
			
			switch(dji_motor[i]->can_info->tx_id){
				case 0x200:
					can_send_num[1][0]=i;
				break;
				case 0x1FF:
					can_send_num[1][1]=i;
				break;
				case 0x2FF:
					can_send_num[1][2]=i;
				break;
			}
        }
    }
	
	if(can_send_num[0][0]>=0){
		canSendMessage(dji_motor[can_send_num[0][0]]->can_info,MotorSendBuffer_can1);
	}
	if(can_send_num[0][1]>=0){
 		canSendMessage(dji_motor[can_send_num[0][1]]->can_info,MotorSendBuffer_can1+8);
	}
	if(can_send_num[0][2]>=0){
		canSendMessage(dji_motor[can_send_num[0][2]]->can_info,MotorSendBuffer_can1+16);
	}
	
	if(can_send_num[1][0]>=0){
		canSendMessage(dji_motor[can_send_num[1][0]]->can_info,MotorSendBuffer_can2);
	}
	if(can_send_num[1][1]>=0){
		canSendMessage(dji_motor[can_send_num[1][1]]->can_info,MotorSendBuffer_can2+8);
	}
	if(can_send_num[1][2]>=0){
		canSendMessage(dji_motor[can_send_num[1][2]]->can_info,MotorSendBuffer_can2+16);
	}
	
    return ret;
}

void djiMotorInfoUpdate(DJI_Motor_t *motor,uint8_t *data)
{
    motor->state_interfaces.ecd = (uint16_t)((data)[0] << 8 | (data)[1]);
    motor->state_interfaces.speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);
    motor->state_interfaces.given_current = (uint16_t)((data)[4] << 8 | (data)[5]);
    motor->state_interfaces.temperate = (data)[6];
    motor->state_interfaces.angle = motor->state_interfaces.ecd/ 8192.0f * 360.0f - 180.0f;
}

void djiMotorSpeedControl(DJI_Motor_t *motor,Speed_Controller_t *controller)
{
    motor->command_interfaces.command =  (int16_t)PIDCalculate(controller->pid, motor->state_interfaces.speed_rpm,motor->command_interfaces.speed_rpm);
}

Speed_Controller_t *speedControllerInit(PID_Init_Config_s *config)
{
    Speed_Controller_t *controller = (Speed_Controller_t *)malloc(sizeof(Speed_Controller_t));
    memset(controller, 0, sizeof(Speed_Controller_t));
    PIDInstance *instance = (PIDInstance *)malloc(sizeof(PIDInstance));
    PIDInit(instance, config);
    controller->pid = instance;

/*
    PIDInit(controller->pid, config);
    这样写为什么不可以
*/

    return controller;
}
Position_Controller_t *positionControllerInit(cascade_PID_Init_Config_s *config,float *out_fdb)
{
    Position_Controller_t *controller = (Position_Controller_t *)malloc(sizeof(Position_Controller_t));
    memset(controller, 0, sizeof(Position_Controller_t));
    cascadePIDInstacne *instance = (cascadePIDInstacne *)malloc(sizeof(cascadePIDInstacne));
    cascadePIDInit(instance,config);
    controller->out_fdb_src = out_fdb;
    controller->cascade_pid = instance;
    return controller;
}

void djiMotorPositionControl(DJI_Motor_t *motor,Position_Controller_t *controller)
{
    motor->command_interfaces.angle = loop_float_constrain(motor->command_interfaces.angle,*controller->out_fdb_src - 180.f ,*controller->out_fdb_src + 180.f);
    motor->command_interfaces.command = cascadePIDCalculate(controller->cascade_pid,*controller->out_fdb_src,motor->state_interfaces.speed_rpm,motor->command_interfaces.angle);

}
void djiMotor_SwitchRing(DJI_Motor_t *motor,Position_Controller_t *controller)
{   
    if(motor->state_interfaces.angle - motor->command_interfaces.angle > 10.0f)
        motor->command_interfaces.angle += 20.0f;
    else if (motor->state_interfaces.angle - motor->command_interfaces.angle < -10.0f)
        motor->command_interfaces.angle -= 20.0f;
    
    djiMotorPositionControl(motor,controller);
}
