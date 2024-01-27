#include "main.h"
#include "CalculateThread.h"
#include "cmsis_os.h"
#include "djiMotor.h"
#include "DMMotor.h"
#include "RM_remote.h"

#include <string.h>


DJI_Motor_t *my_motor;
DM_Motor_t *dm_motor;


Speed_Controller_t *my_controller;
PID_Init_Config_s my_pid;
RM_Remote_t *my_remote;


void CalculateThread(void const * argument)
{
	osDelay(1000);
	
	DJI_Motor_Register_t motor_reg;
	motor_reg.hcan=&hcan1;
	motor_reg.motor_type=DJI_MOTOR_6020;
	motor_reg.id=1;
	
	memset(&my_pid,0,sizeof(PID_Init_Config_s));
	my_pid.Kp = 20;
	my_pid.Ki = 0;
	my_pid.Kd = 0;
	my_pid.MaxOut = 0x7fff;
	my_pid.Improve = PID_IMPROVE_NONE;
	my_motor = djiMotorAdd(&motor_reg);
	my_controller = speedControllerInit(&my_pid);
	
	DM_Motor_Register_t dm_motor_reg;
	dm_motor_reg.hcan=&hcan1;
	dm_motor_reg.kd_max=5;
	dm_motor_reg.kd_min=0;
	dm_motor_reg.kp_max=500;
	dm_motor_reg.kp_min=0;
	dm_motor_reg.motor_type=DM_MOTOR_4310;
	dm_motor_reg.p_max=12.5;
	dm_motor_reg.v_max=30;
	dm_motor_reg.t_max=10;
	dm_motor_reg.rx_id=0x01E;
	dm_motor_reg.tx_id=0x10E;
	
	dm_motor = dmMotorAdd(&dm_motor_reg);
	
	my_remote = rmRemoteAdd(&huart3);
	while(1)
	{
		my_motor->command_interfaces.speed_rpm = my_remote->state_interfaces.rc.ch[0];
//		my_motor->command_interfaces.command=2000;
		djiMotorSpeedControl(my_motor,my_controller);


		
		djiMotorSendMessage();
		dmMotorSendMessage(dm_motor);
		osDelay(1);
	}

}