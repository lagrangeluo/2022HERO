/**
  ******************************************************************************
  * @file    Project/APP/chassis_task.c 
  * @author  NEUQRM
  * @version V1.0.0
  * @date    12.2021
  * @brief   该文件是战车输出任务汇总
  ******************************************************************************
  * @attention
  ******************************************************************************
  ****************************(C) COPYRIGHT 2021 NEUQRM****************************

  ...........  革命尚未成功，同志仍需努力  ...........
*/

#include "chassis_task.h"
#include "bsp_can.h"
#include "supercap.h"
#include "referee_UI.h"

void CHASSIS_TASK()//TIM3定时器中断控制战车（已经过宏定义）
{
	static int time_count=1;
	time_count++;
	
//	remote_control();	
	if(time_count%7==0)
	{
		//底盘移动任务
		chassis_move();
		referee_unpack_fifo_data();
		
	}
	if(time_count%2==0)
	{
		//yaw轴旋转任务
		yaw_turn();
	}	
	if(time_count%3==0)
	{
		//拨弹轮旋转任务
		trigger_turn();
		send_gimbal_data_2();
	}	
//	if(time_count%60==0)
//		//超级电容
//		supercap();
	
	if(time_count%400==0)
		UI_Display();
	if(time_count>=1000)			//清除计数标志    1s
	{time_count=1;}
}

