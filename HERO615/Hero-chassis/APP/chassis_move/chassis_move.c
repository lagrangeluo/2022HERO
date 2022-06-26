/**
  ******************************************************************************
  * @file    Project/APP/chassis_move.c 
  * @author  NEUQRM
  * @version V1.0.0
  * @date    12.2021
  * @brief   该文件是针对得到的模式来进行对应的输出和底盘功率限制
  ******************************************************************************
  * @attention
  ******************************************************************************
  ****************************(C) COPYRIGHT 2021 NEUQRM****************************

  ...........  革命尚未成功，同志仍需努力  ...........
*/  

#include "chassis_move.h"

void chassis_spin(float *vx,float *vy); 
static float chassis_follow(void);
static void chassis_speed_control(float speed_x, float speed_y, float speed_r);
static void chassis_move_mode(void);
static void can_send_chassis_current(void);
static float Get_chassis_theta(void);
static void power_limitation_jugement(void);
static void power_limitation_jugement(void);
static float chassis_buffer_loop(uint16_t buffer);
static void chassis_fly(uint16_t buffer);
static float chassis_power_loop(uint16_t target_power,float actual_power,float last_power);

float total_current_limit,total_current,power;
uint16_t buffer,max_power;
BUFFER_PID_t b_pid;
float current_scale;

CHASSIS_CONTROL_ORDER_t chassis_control_order;
MOTOR_t chassis_motor1,chassis_motor2,chassis_motor3,chassis_motor4,chassis_center;
POWER_PID_t p_pid;
int shoot_flag=0;

POWER_PID_t p_pid;
BUFFER_PID_t b_pid;
REAl_CHASSIS_SPEED_t real_chassis_speed;
int8_t max_d_speed_x;
int8_t max_d_speed_y;
float avx,avy,awz;
float last_vx,last_vy;

void chassis_move(void)//底盘移动过程
{
	//对应模式的底盘状态赋值
	chassis_move_mode();
	//功率判断与限制
	power_limitation_jugement();
	//pid计算
	vpid_chassis_realize();
	
	//输出底盘电流值
	can_send_chassis_current();
}
int flag3=0;
static void chassis_speed_control(float speed_x, float speed_y, float speed_r)
{
	flag3=1;
	int max;
		//速度换算
		BaseVel_To_WheelVel(speed_x, speed_y, speed_r);
 

		max=find_max();
		if(max>MAX_MOTOR_SPEED)
		{
			chassis_motor1.target_speed=(int)(chassis_motor1.target_speed*MAX_MOTOR_SPEED*1.0/max);
			chassis_motor2.target_speed=(int)(chassis_motor2.target_speed*MAX_MOTOR_SPEED*1.0/max);
			chassis_motor3.target_speed=(int)(chassis_motor3.target_speed*MAX_MOTOR_SPEED*1.0/max);
			chassis_motor4.target_speed=(int)(chassis_motor4.target_speed*MAX_MOTOR_SPEED*1.0/max);
		}
	set_chassis_speed(chassis_motor1.target_speed, chassis_motor2.target_speed, chassis_motor3.target_speed, chassis_motor4.target_speed);
}	

static float chassis_follow(void)
{
	chassis_center.pid.position_loop.apid.target_angle=GIMBAL_HEAD_ANGLE;
	chassis_center.pid.position_loop.apid.actual_angle=chassis_center.actual_angle;
	follow_pid_realize();
	return (float)chassis_center.pid.position_loop.apid.PID_OUT;
}
STEPSTAR step_flag;
float K_VX,K_VY,B_VX,B_VY; //分别代表K和B 一次函数
int step_times_x=0,step_times_y=0; //时间
float TIME_LIMIT_X=180,TIME_LIMIT_Y=280; //斜坡的时间 
int STEP_VALUE=50; //差值大于step_value就用斜坡
//斜坡函数状态判断
void step_flag_judge(float VX_,float VY_,float LAST_VX_,float LAST_VY_)
{
	if(step_flag==NO_STEP)
	{
		if(abs(VX_-LAST_VX_)>STEP_VALUE) step_flag=X_STEP;
	    else if(abs(VY_-LAST_VY_)>STEP_VALUE*0.4) step_flag=Y_STEP;
		return;
	}
	if(step_flag==X_STEP)
	{
		if(step_times_x>TIME_LIMIT_X)
	    {
		    step_times_x=0;
		    step_flag=NO_STEP;
			return;
	    }
		 if(abs(VX_)<=1.0f) 
	     {
		    step_flag=NO_STEP;
		    step_times_x=0;
			return;
	     }
		if(abs(VY_-LAST_VY_)>STEP_VALUE&&abs(VY_)>90) 
		{
			step_flag=XY_STEP;
		}
		return;
	}
	
	if(step_flag==Y_STEP)
	{
		if(step_times_y>TIME_LIMIT_Y)
	    {
		    step_times_y=0;
		    step_flag=NO_STEP;
			return;
	    }
		 if(abs(VY_)<=1.0f) 
	     {
		    step_flag=NO_STEP;
		    step_times_y=0;
			return;
	     }
		if(abs(VX_-LAST_VX_)>STEP_VALUE&&abs(VX_)>90) 
		{
			step_flag=XY_STEP;
		}
		return;
	}
	
	if(step_flag==XY_STEP)
	{
		if(step_times_y>TIME_LIMIT_Y &&step_times_x>TIME_LIMIT_X)
	    {
		    step_times_y=0;
			step_times_x=0;
		    step_flag=NO_STEP;
			return;
	    }
		if(step_times_x>TIME_LIMIT_X)
	    {
		    step_times_x=0;
		    step_flag=Y_STEP;
			return;
	    }
		if(step_times_y>TIME_LIMIT_Y)
	    {
		    step_times_y=0;
		    step_flag=X_STEP;
			return;
	    }
		 if(abs(VY_)<=1.0f&&abs(VX_)<=1.0f) 
	     {
		    step_flag=NO_STEP;
		    step_times_y=0;
			 step_times_x=0;
			 return;
	     }
		 if(abs(VY_)<=1.0f) 
	     {
		    step_flag=X_STEP;
		    step_times_y=0;
	     }
		 if(abs(VX_)<=1.0f) 
	     {
		    step_flag=Y_STEP;
		    step_times_x=0;
	     }
		return;
	}
	
	
}

void step_star(float *VX_,float *VY_,float LAST_VX_,float LAST_VY_)
{
	step_flag_judge(*VX_,*VY_,LAST_VX_,LAST_VY_);

	if(step_flag==NO_STEP)  return;

	
	if(step_flag==X_STEP)
	{
		step_times_x++;
		if(step_times_x<=1)
		{
			K_VX=(*VX_-LAST_VX_)/TIME_LIMIT_X;
			B_VX=LAST_VX_;
		}
		
		*VX_=(float)(K_VX*(float)step_times_x)+B_VX;
		
	}
	if(step_flag==Y_STEP)
	{
		step_times_y++;
		if(step_times_y<=1)
		{
			K_VY=(*VY_-LAST_VY_)/TIME_LIMIT_Y;
			B_VY=LAST_VY_;
		}
		
		*VY_=K_VY*(float)step_times_y+B_VY;
	}
	if(step_flag==XY_STEP)
	{
		step_times_y++;
		if(step_times_y<=1)
		{
			K_VY=(*VY_-LAST_VY_)/TIME_LIMIT_Y;
			B_VY=LAST_VY_;
		}
		step_times_x++;
		if(step_times_x<=1)
		{
			K_VX=(*VX_-LAST_VX_)/TIME_LIMIT_X;
			B_VX=LAST_VX_;
		}
		
		*VX_=(float)(K_VX*(float)step_times_x)+B_VX;
		
		*VY_=K_VY*(float)step_times_y+B_VY;
	}
}
float vx,vy,wz;

float yu;
static void chassis_move_mode(void)//对应模式下的底盘动作
{
	
	vx=(float)chassis_control_order.vx_set;
	vy=(float)chassis_control_order.vy_set;
	wz=(float)chassis_control_order.wz_set;
//	vx=yu;
	avx=vx;
	avy=vy;
	step_star(&avx,&avy,last_vx,last_vy);
	if(step_flag!=NO_STEP)
	{
		vx=avx;
	    vy=avy;
	}
//	chassis_control_order.chassis_mode=CHASSIS_REMOTE_CLOSE;

	if(chassis_control_order.chassis_mode==CHASSIS_REMOTE_CLOSE)
	{
	}
  else if(chassis_control_order.chassis_mode==CHASSIS_NORMAL)
	{
		chassis_spin(&vx,&vy);
		wz = -1.0f*chassis_follow();
	}
	else if(chassis_control_order.chassis_mode==CHASSIS_SPIN)
	{
		chassis_spin(&vx,&vy);
		wz=1.5;
	}
	last_vx=(float)chassis_control_order.vx_set;
//	last_vx=yu;
	last_vy=(float)chassis_control_order.vy_set;
	chassis_speed_control(vx,vy,wz);
}
int flag1=0;
static void can_send_chassis_current(void)//输出底盘电流值
{
	flag1=1;
	static uint8_t cdata[8];
	cdata[0]=(chassis_motor1.pid.speed_loop.vpid.PID_OUT)>>8;
	cdata[1]=(chassis_motor1.pid.speed_loop.vpid.PID_OUT)&0xFF;
	cdata[2]=(chassis_motor2.pid.speed_loop.vpid.PID_OUT)>>8;
	cdata[3]=(chassis_motor2.pid.speed_loop.vpid.PID_OUT)&0xFF;
	cdata[4]=(chassis_motor3.pid.speed_loop.vpid.PID_OUT)>>8;
	cdata[5]=(chassis_motor3.pid.speed_loop.vpid.PID_OUT)&0xFF;
	cdata[6]=(chassis_motor4.pid.speed_loop.vpid.PID_OUT)>>8;
	cdata[7]=(chassis_motor4.pid.speed_loop.vpid.PID_OUT)&0xFF;
	
	Can_Tx_Message(&hcan1,cdata);
}

float theta;       
void chassis_spin(float *vx,float *vy) 
{
	theta=Get_chassis_theta();   
	*vx = (float)(avy*sin(theta) + avx*cos(theta));    
	*vy = (float)(avy*cos(theta) - avx*sin(theta));    
}

static float Get_chassis_theta(void)//读取底盘相对云台转角（需要陀螺仪）
{
	float temp,angle;
	if(chassis_center.actual_angle<chassis_center.switch_mode_angle)
		angle=chassis_center.actual_angle+360.0f;
	else
		angle=chassis_center.actual_angle;
	temp=angle-chassis_center.switch_mode_angle;	
	angle=temp/360.0f*2*PI;
	return angle;
}

/**
  * @breif         底盘功率限制
  * @param[in]     none 
	* @param[out]    输出限制后的四个电机电流值
  * @retval        none     
  */
float current_scale,BUFFER_MAX=60.0f,POWER_TOTAL_CURRENT_LIMIT=9000.0f;
float temp3,temp1,temp2,speed1,speed2,speed3,speed4,total_current_limit,total_current,power,last_power;
float power_scale,buffer_scale;
uint8_t fly_flag;
 static void power_limitation_jugement(void)
{
	 total_current=0;
	last_power=power;
	get_chassis_power_and_buffer_and_max(&power,&buffer,&max_power);
	
	

	power_scale=chassis_power_loop(max_power-8,power,last_power);
	buffer_scale=chassis_buffer_loop(buffer);
	temp1=chassis_motor1.pid.speed_loop.vpid.target_speed*buffer_scale*power_scale;
	chassis_motor1.pid.speed_loop.vpid.target_speed=(int16_t)temp1;
	temp1=chassis_motor2.pid.speed_loop.vpid.target_speed*buffer_scale*power_scale;
	chassis_motor2.pid.speed_loop.vpid.target_speed=(int16_t)temp1;
	temp1=chassis_motor3.pid.speed_loop.vpid.target_speed*buffer_scale*power_scale;
	chassis_motor3.pid.speed_loop.vpid.target_speed=(int16_t)temp1;
	temp1=chassis_motor4.pid.speed_loop.vpid.target_speed*buffer_scale*power_scale;
	chassis_motor4.pid.speed_loop.vpid.target_speed=(int16_t)temp1;
//	if(power>max_power*2)CHASSIS_vPID_max=5000;
//	else if(power<max_power*1.6)CHASSIS_vPID_max=8000;
	if(buffer<BUFFER_MAX*0.15) CHASSIS_vPID_max=500;
	if(buffer<BUFFER_MAX*0.25&& buffer>=BUFFER_MAX*0.15) CHASSIS_vPID_max=900;
	if(buffer<BUFFER_MAX*0.36&&CHASSIS_vPID_max<=1200) CHASSIS_vPID_max=1200;
	if(CHASSIS_vPID_max<=1900 && buffer>=BUFFER_MAX*0.55f ) CHASSIS_vPID_max=1600;
	if(CHASSIS_vPID_max<=2100 && buffer>=BUFFER_MAX*0.75f ) CHASSIS_vPID_max=3200;
	if(CHASSIS_vPID_max<=4500 && buffer>=BUFFER_MAX*0.96f ) CHASSIS_vPID_max=8000;
}
/**
  * @breif         底盘功率环函数
  * @param[in]     target_power：设定的目标值
	* @param[in]     target_power：返回的真实值  
	* @param[in]     last_power：上一次返回的真实值
	* @param[out]    四个电机的输出电流
  * @retval        none     
  */
static float chassis_power_loop(uint16_t target_power,float actual_power,float last_power)
{
	float temp;
	p_pid.target_power=(float)target_power;
	p_pid.actual_power=actual_power;
	//此处进行pid运算
	power_pid_realize(&p_pid);
	//此处计算比例系数
	temp=1.07+((float)p_pid.PID_OUT/1000);
//	if(temp>1.2f)  temp-=0.2f;
//	temp=temp/2.0*0.45f+0.1f;
	temp*=0.7;

	return temp;
}

static float chassis_buffer_loop(uint16_t buffer)
{
	float temp;
	b_pid.target_buffer=50;
	b_pid.actual_buffer=buffer;
	buffer_pid_realize(&b_pid);
	temp=1.07-((float)b_pid.PID_OUT/1000.0f);
	temp*=0.75;
//	if(temp>1.2f)  temp-=0.2f;

	return temp;
}
/**
  * @breif         底盘飞坡函数，防止因飞坡后缓冲能量用完
  * @param[in]     buffer：底盘缓冲能量
	* @param[out]    四个电机的输出电流
  * @retval        none     
  */
static void chassis_fly(uint16_t buffer)
{
	if(buffer<20)
	{
		chassis_motor1.pid.speed_loop.vpid.PID_OUT*=0.5f;
		chassis_motor2.pid.speed_loop.vpid.PID_OUT*=0.5f;
		chassis_motor3.pid.speed_loop.vpid.PID_OUT*=0.5f;
		chassis_motor4.pid.speed_loop.vpid.PID_OUT*=0.5f;
	}
}
