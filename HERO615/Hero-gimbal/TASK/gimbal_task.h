#ifndef __gimbal_task_H
#define __gimbal_task_H

#include "pid.h"
#include "math.h"
#include "remote_control.h"
#include "mode_control.h"
#include "stm32f4xx.h"
//��̨��ʼ������ֵ����������,��������Χ��ֹͣһ��ʱ���Լ����ʱ��6s������ʼ��״̬��
#define GIMBAL_INIT_ANGLE_ERROR 3.7f
#define GIMBAL_INIT_STOP_TIME 100
#define GIMBAL_INIT_TIME 6000

//��̨��ʼ������ֵ���ٶ��Լ����Ƶ��ĽǶ�
#define GIMBAL_INIT_PITCH_SPEED 0.004f
#define GIMBAL_INIT_YAW_SPEED   0.005f
#define INIT_YAW_SET 0.0f
#define INIT_PITCH_SET 0.0f

//��̨�Ƕȷ�Χ
#define YAW_ANGLE_MAX 360.0f
#define YAW_ANGLE_MIN 0.0f

#define PITCH_ANGLE_MAX 360.0f
#define PITCH_ANGLE_MIN 0.0f





typedef struct{
	
	PidTypeDef gimbal_raw_pid;
	PidTypeDef gimbal_gyro_pid;
	PidTypeDef gimbal_enconde_pid_speed;//�ٶ�
	PidTypeDef gimbal_enconde_pid;//�Ƕ�
	//λ��ʽ
	float target_angle;
	float IMU_actual_angle;
	   
	float target_speed;
	float IMU_actual_speed;
	
	float CAN_actual_speed;
	float CAN_actual_angle;
	float CAN_total_angle;

	float add_angle;
	
	int16_t given_current;
	
	MODE_t gimbal_motor_mode;
}GIMBAL_t;

extern GIMBAL_t gimbal_y,gimbal_p;
extern GIMBAL_MODE_t gimbal_set_mode;

void Start_UP_Forward(void);
void Start_UP_Backward(void);
void Gimbal_Task(void);
void Gimbal_Init(void);

#endif
