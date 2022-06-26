/**
  ******************************************************************************
  * @file    Project/Application/User/Core/my_init.c 
  * @author  NEUQRM
  * @version V1.0.0
  * @date    12.2021
  * @brief   所有初始化函数调用，一并进行初始化
  ******************************************************************************
  * @attention
  ******************************************************************************
  ****************************(C) COPYRIGHT 2021 NEUQRM****************************

  ...........  革命尚未成功，同志仍需努力  ...........
*/
#include "my_init.h"

void mode_init(void)
{
	chassis_center.pid.loop_flag=POSITION_LOOP;
	chassis_motor1.pid.loop_flag=SPEED_LOOP;
	chassis_motor2.pid.loop_flag=SPEED_LOOP;
	chassis_motor3.pid.loop_flag=SPEED_LOOP;
	chassis_motor4.pid.loop_flag=SPEED_LOOP;
	
}
void all_init(void)
{
	motor_pid_init(&(chassis_center.pid));
	motor_pid_init(&(chassis_motor1.pid));
	motor_pid_init(&(chassis_motor2.pid));
	motor_pid_init(&(chassis_motor3.pid));
	motor_pid_init(&(chassis_motor4.pid));
	pid_init();
	trigger_all_init();
	referee_usart_fifo_init();
	POWER_PID_Init(&p_pid);//目前无用
	remote_control_init();
	bsp_can_init();
	HAL_TIM_Base_Start_IT(&htim3);
	init_referee_struct_data();
	Yaw_PIDinit();
//	Gimbal_Init();
}

