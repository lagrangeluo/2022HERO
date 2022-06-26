#ifndef __can_receive_H
#define __can_receive_H

#include "main.h"

#include "bsp_can.h"
//#define PITCH_MOTOR_ID				 0x206
#define PITCH_MOTOR_ID				 0x205
#define SHOOT_LEFT_MOTOR_ID    0x201
#define SHOOT_RIGH_MOTOR_ID    0x202
#define SHOOT_MOTOR_TRIGGER_ID 0x207
#define YAW_ID    0x20B

typedef struct
{
	uint16_t    lastangle; /*�ϴε���Ƕ�*/
	
	uint16_t 		sumangle;/*�ǶȺ�*/
	
	uint16_t 		angle;/*���ʵ�ʽǶ�*/
	
	int16_t		  speed;/*���ʵʱ�ٶ�*/
	
	uint8_t     temp; /*����¶�*/
	
	int32_t 		turns;/*������ϵ翪ʼת����Ȧ��*/
}Motor_HandleTypeDef;		


extern Motor_HandleTypeDef yaw_can_rx,pitch_can_rx,shoot_can_rx[2];
extern int shoot_flag;


#endif