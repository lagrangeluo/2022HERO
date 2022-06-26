/**
  ******************************************************************************
  * @file    Project/APP/trigger_turn.c 
  * @author  NEUQRM
  * @version V1.0.0
  * @date    12.2021
  * @brief   该文件包含了拨弹轮的pid参数值，赋值，pid运算，输出等一系列跟拨弹轮有关的函数
  ******************************************************************************
  * @attention
  ******************************************************************************
  ****************************(C) COPYRIGHT 2021 NEUQRM****************************

  ...........  革命尚未成功，同志仍需努力  ...........
*/

#include "trigger_turn.h"



trigger_t trigger;

float TriggerSpeed[5] 	= {11.2f,			0.5f,			3.0f,		30000,			5000};  //拨弹轮速度环
float TriggerAngle[5] 	= {0.14f,			0.0f,			0.000f,		3200,			0.0f};  //拨弹轮角度环

//更新发子弹个数
void shoot_angle_clc(void);
//拨弹轮PID计算
static void trigger_pid(void);
//拨弹轮附带yaw轴任务
void trigger_turn(void)
{
//shoot_angle_clc();    //在接收云台对应指令时执行该函数（这里不执行）
	if(shoot_flag>=1)
	{
		shoot_flag++;
	}
	if(shoot_flag>=150)
	{
	   trigger.target_angle-=((17.5*360/7.f)*8191/360);
	   shoot_flag=0;
	}
	trigger_pid();
	canTX_trigger(trigger.set_currunt);
}

void trigger_all_init(void)//拨弹轮pid参数及状态初始化
{
	PID_Init(&trigger.speed_pid,TriggerSpeed);
	PID_Init(&trigger.angle_pid,TriggerAngle);
	
	trigger.actual_angle = 0.0f;
	trigger.actual_speed = 0.0f;
	
	trigger.last_shoot_flag = 0;
	
	trigger.target_angle = 13.0f;
	trigger.target_speed = 0.0f;
}

void shoot_angle_clc(void)
{
	float target_angle;
	trigger.last_shoot_flag=1;
	if(trigger.rounds>=400)
	{
			trigger.target_angle-=(trigger.rounds-3)*8192;
			if(trigger.target_angle<=0) trigger.target_angle=1;
			trigger.total_angle -=(trigger.rounds-3)*8192 ;
			trigger.begin_angle=trigger.total_angle;
			trigger.rounds=3;
	}
	target_angle=((SHOOT_NUM*38.5*360/7.f)*8191/360);          //发射1发
	target_angle+=((9.5*360/7.f)*8191/360);
	trigger.target_angle+=target_angle;
}

static void trigger_pid(void)//拨弹轮pid赋值计算
{
	apid_vpid_realize(&trigger.angle_pid,trigger.target_angle,trigger.total_angle);
	
	trigger.target_speed = trigger.angle_pid.out;
	
	apid_vpid_realize(&trigger.speed_pid,trigger.target_speed,trigger.actual_speed);
	
	trigger.set_currunt = trigger.speed_pid.out;
}


uint8_t canTX_trigger(int16_t trigger)//拨弹轮电流输出
{
	CAN_TxHeaderTypeDef canFrame;
	uint8_t data[8]={0};
	uint32_t temp=0;
	
	canFrame.IDE=CAN_ID_STD;
	canFrame.StdId=0x1ff;
	canFrame.RTR=CAN_RTR_DATA;
	canFrame.DLC=8;
	canFrame.TransmitGlobalTime=DISABLE;
	data[0]=0;
	data[1]=0;
	data[2]=0;
	data[3]=0;
	data[4]=0;
	data[5]=0;
	data[6]=trigger>>8;
	data[7]=trigger&0xff;
	HAL_CAN_AddTxMessage(&hcan1, &canFrame, data,&temp);
	
	return temp;
}

void Trigger_Motor_Callback(trigger_t *motor,uint16_t angle, int16_t speed)//拨弹轮电调回调
{
	motor->last_angle = motor->actual_angle;
	motor->actual_angle = angle;
	motor->actual_speed = 0.5*(speed + motor->last_speed);
	motor->last_speed = speed;

	if(motor->record_begin_angle_status==0)
	{
		motor->begin_angle = angle;
		motor->record_begin_angle_status++;
	}
	if(motor->actual_angle - motor->last_angle > 4096)
		motor->rounds --;
	else if (motor->actual_angle - motor->last_angle < -4096)
		motor->rounds ++;
	motor->total_angle = motor->rounds * 8192 + motor->actual_angle;

	
}
