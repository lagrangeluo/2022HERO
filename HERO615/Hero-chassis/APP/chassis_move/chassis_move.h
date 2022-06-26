#ifndef __MOTOR_H
#define __MOTOR_H

#include "pid.h"
#include "kinematics.h"
#include "can.h"
#include <math.h>
#include "referee.h"

//Ҳ������ֱ̨�Ӵ���ָ��
#define CHASSIS_REMOTE_CLOSE   	 1    //�ر�ң����                                ���λ
#define CHASSIS_NORMAL           3    //����ģʽ                                  ����λ
#define CHASSIS_SPIN             2    //С����ģʽ                                    ���λ

#define GIMBAL_HEAD_ANGLE  100   

#define POWER_LIMIT         80.0f
#define WARNING_POWER       40.0f   
#define WARNING_POWER_BUFF  50.0f  

#define follow_pid_realize() \
				do{ \
						switch_flag=FOLLOW;      \
						pid_realize(&(chassis_center.pid));   \
					  switch_flag=NUL;  \
				}while(0)
				
#define vpid_chassis_realize() \
				do{ \
						switch_flag=CHASSIS;      \
						pid_realize(&(chassis_motor1.pid));   \
						pid_realize(&(chassis_motor2.pid));   \
						pid_realize(&(chassis_motor3.pid));   \
						pid_realize(&(chassis_motor4.pid));   \
					  switch_flag=NUL;  \
				}while(0)
				

			
typedef struct{
	
	float start_angle;			//�����ʼ�Ƕ�ֵ
	int start_angle_flag;	//��¼�����ʼ�Ƕ�ֵ��flag
	int switch_mode_flag;  //��¼ģʽת���Ƕ�ֵ��flag
	int stop_angle;				//����ֹͣ����ʱ��ĽǶ�ֵ
	float target_angle;
	
	float actual_angle;			//��ǰ��ʵ�Ƕ�ֵ
	float last_angle;				//��һ�η��صĽǶ�ֵ
	float switch_mode_angle;  //��¼ģʽת���Ƕ�ֵ
	int round_cnt;				//��Կ���ʱת����Ȧ��
	int total_angle;			//�ܹ�ת���ļ���
	
	float actual_speed;			//�����ʵ�ٶ�,rpm
	int target_speed;			//���Ŀ���ٶ�,rpm  ת/min
	int last_speed;       //�����һ�λش����ٶ�ֵ
	int actual_current;		//�����ʵ����
	int target_current;		//���Ŀ�����
	//int temp;							//����¶ȣ�2006�����֧�֣�3508֧�֣�
	PID_t pid;
}MOTOR_t;

typedef struct
{
	int16_t vx_set;
	int16_t vy_set;
	int16_t wz_set;
	uint8_t chassis_mode;
	uint8_t last_chassis_mode;
	uint16_t gimbal_6020_angle;
}CHASSIS_CONTROL_ORDER_t;

typedef struct
{
	int16_t real_vx;
	int16_t real_vy;
}REAl_CHASSIS_SPEED_t;

typedef enum
{
	NO_STEP=0,
	X_STEP,
	Y_STEP,	
	XY_STEP,
}STEPSTAR;


extern MOTOR_t chassis_motor1,chassis_motor2,chassis_motor3,chassis_motor4,chassis_center;
extern CHASSIS_CONTROL_ORDER_t chassis_control_order;
extern POWER_PID_t p_pid;
extern BUFFER_PID_t b_pid;
extern int shoot_flag;
void chassis_move(void);
#endif

