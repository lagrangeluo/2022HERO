#include "can_receive.h"
#include "stm32f4xx.h"
#include "gimbal_task.h"
#include "string.h"
#include "math.h"
#include "led.h"
#include "shoot_task.h"


Motor_HandleTypeDef	yaw_can_rx={0},pitch_can_rx={0},shoot_can_rx[2]={0};

int16_t euler_angle[3];
int shoot_flag=0;
/**
  * @brief          can总线的接收
  * @param[*hcan]   CAN_HandleTypeDef类型的指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rxFrame;
	uint8_t rxData[8]={0};
	led_on(&led[2]);
	if(hcan==&hcan1)
	{
		
		HAL_CAN_GetRxMessage(&hcan1,  CAN_RX_FIFO0, &rxFrame,  rxData);
		switch (rxFrame.StdId)
		{
//			case SHOOT_MOTOR_TRIGGER_ID:
//								{
//										Trigger_Motor_Callback(&rc_shoot.trigger,(int16_t)rxData[0]<<8|rxData[1],(int16_t) rxData[2]<<8|rxData[3]);
//								}
			case PITCH_MOTOR_ID:
								{
										pitch_can_rx.angle = (int16_t) rxData[0]<<8|rxData[1];
										pitch_can_rx.speed = (int16_t) rxData[2]<<8|rxData[3];
										pitch_can_rx.temp  = (int16_t) rxData[6];
										break;
								}	
			case SHOOT_LEFT_MOTOR_ID:
								{
										shoot_can_rx[0].angle = (int16_t) rxData[0]<<8|rxData[1];	
										shoot_can_rx[0].speed = (int16_t) rxData[2]<<8|rxData[3];
										rc_shoot.left_fric.actual_speed =shoot_can_rx[0].speed ;
										break;
								}	
			case SHOOT_RIGH_MOTOR_ID:
								{
										shoot_can_rx[1].angle = (int16_t) rxData[0]<<8|rxData[1];	
										shoot_can_rx[1].speed = (int16_t) rxData[2]<<8|rxData[3];
										rc_shoot.right_fric.actual_speed=shoot_can_rx[1].speed ;
										break;
								}									
		}
	}
	else if(hcan==&hcan2)
	{
		HAL_CAN_GetRxMessage(&hcan2,  CAN_RX_FIFO0, &rxFrame,  rxData);
		switch (rxFrame.StdId)
		{
				case YAW_ID:
								{       yaw_can_rx.lastangle = yaw_can_rx.angle;
										yaw_can_rx.angle = (int16_t) rxData[0]<<8|rxData[1];
										yaw_can_rx.speed = (int16_t) rxData[2]<<8|rxData[3];
									  shoot_flag= (int16_t) rxData[4]<<8|rxData[5];

									
									if(gimbal_set_mode != GIMBAL_TOP_ANGLE)
									{
										 yaw_can_rx.turns=(yaw_can_rx.angle-yaw_can_rx.lastangle)<-150?yaw_can_rx.turns+1:yaw_can_rx.turns;
										yaw_can_rx.turns=(yaw_can_rx.angle-yaw_can_rx.lastangle)>150? yaw_can_rx.turns-1:yaw_can_rx.turns;	
										
									}
									   break;
								}																	
		}		
	}
}
