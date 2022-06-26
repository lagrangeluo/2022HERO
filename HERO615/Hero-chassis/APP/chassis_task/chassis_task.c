/**
  ******************************************************************************
  * @file    Project/APP/chassis_task.c 
  * @author  NEUQRM
  * @version V1.0.0
  * @date    12.2021
  * @brief   ���ļ���ս������������
  ******************************************************************************
  * @attention
  ******************************************************************************
  ****************************(C) COPYRIGHT 2021 NEUQRM****************************

  ...........  ������δ�ɹ���ͬ־����Ŭ��  ...........
*/

#include "chassis_task.h"
#include "bsp_can.h"
#include "supercap.h"
#include "referee_UI.h"

void CHASSIS_TASK()//TIM3��ʱ���жϿ���ս�����Ѿ����궨�壩
{
	static int time_count=1;
	time_count++;
	
//	remote_control();	
	if(time_count%7==0)
	{
		//�����ƶ�����
		chassis_move();
		referee_unpack_fifo_data();
		
	}
	if(time_count%2==0)
	{
		//yaw����ת����
		yaw_turn();
	}	
	if(time_count%3==0)
	{
		//��������ת����
		trigger_turn();
		send_gimbal_data_2();
	}	
//	if(time_count%60==0)
//		//��������
//		supercap();
	
	if(time_count%400==0)
		UI_Display();
	if(time_count>=1000)			//���������־    1s
	{time_count=1;}
}

