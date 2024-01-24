#include "main.h"
#include "CalculateThread.h"
#include "cmsis_os.h"
#include "djiMotor.h"


DJI_Motor_t *my_motor;

Speed_Controller_t *my_controller;
PID_Init_Config_s my_pid;



void CalculateThread(void const * argument)
{
	DJI_Motor_Register_t motor_reg;
	motor_reg.hcan=&hcan1;
	motor_reg.motor_type=MOTOR_6020;
	motor_reg.id=1;
	
	memset(&my_pid,0,sizeof(PID_Init_Config_s));
	my_pid.Kp = 20;
	my_pid.Ki = 0;
	my_pid.Kd = 0;
	my_pid.MaxOut = 0x7fff;
	my_pid.Improve = PID_IMPROVE_NONE;
	my_motor = djiMotorAdd(&motor_reg);
	my_motor->command_interfaces.speed_rpm = 20;
	my_controller = speedControllerInit(&my_pid);
	while(1)
	{
		
//		my_motor->command_interfaces.command=2000;
		djiMotorSpeedControl(my_motor,my_controller);


		
		djiMotorSendMessage();
		osDelay(1);
	}

}