#include "main.h"
#include "CalculateThread.h"
#include "cmsis_os.h"
#include "djiMotor.h"

DJI_Motor_t *my_motor;
Speed_Controller_t *my_controller;
PID_Init_Config_s my_pid;

void CalculateThread(void const * argument)
{
	memset(&my_pid,0,sizeof(PID_Init_Config_s));
	my_pid.Kp = 20;
	my_pid.Ki = 0;
	my_pid.Kd = 0;
	my_pid.MaxOut = 30000;
	my_motor = djiMotorAdd(1, 0, &hcan1);
	my_motor->command_interfaces.speed_rpm = 20;
	my_controller = speedControllerInit(&my_pid);
	while(1)
	{
		

		djiMotorSpeedControl(my_motor,my_controller);
		djiMotorSendMessage();
		osDelay(1);
	}

}