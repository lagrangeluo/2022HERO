/**
  ******************************************************************************
  * @file    Project/APP/vision.c 
  * @author  NEUQRM
  * @version V1.0.0
  * @date    12.2021
  * @brief   �Ƿ�����Ӿ�ģʽ����Ϊ����Ҫ������̨�����ݣ����������λ����ֱ�ӽӵ������ϸ��ļ��Ϳ��Ժ��ԣ�
  ******************************************************************************
  * @attention
  ******************************************************************************
  ****************************(C) COPYRIGHT 2021 NEUQRM****************************

  ...........  ������δ�ɹ���ͬ־����Ŭ��  ...........
*/

#include "vision.h"

VISION_t vision_mode=VISION_OFF;
VISION_GET_t vision_sent;

//void Vision_Task(void)
//{
//	if(vision_mode==VISION_OFF)
//	{
//		gimbal_y.add_angle=rc_sent.yaw.target_angle;
//		gimbal_y.target_speed=rc_sent.yaw.target_speed*0.5f;
//	}
//	else
//	{
//		gimbal_y.add_angle=vision_sent.yaw.target_angle;
//	}
//}

