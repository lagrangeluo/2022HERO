#include "gimbal_task.h"
#include "vision_task.h"
#include "gimbal_task_behaviour.h"
#include "sent_task.h"
#include "bsp_imu.h"
#include "can_receive.h"
#include "RC_task.h"
#define yaw_angle gimbal_y.add_angle
#define pitch_angle gimbal_p.add_angle

GIMBAL_t gimbal_y,gimbal_p;



GIMBAL_MODE_t gimbal_set_mode;
// 																KP 				KI 			 KD 	 MAX_OUT   MAX_IOUT  deadband
float YawGyroPid[6] 		= {0.185f,	0.0f,	0.0f,	1.9f,	0.0f ,0.0f};  	//imu角度环
float YawEncondePid[6] 		= {0.2f,	0.0f,	0.0f,	1.4f,	0.0f,   0.0f};  	//编码器角度环
float YawEncondePidSpeed[6]	= {0.0f,	0.0f,	0.0f,	0.0f,	0.0f,	0.0f};  	//编码器角度环
float YawSpeedPid[6] 		= {14800.0f,	100.0f,		500.0f,	22000.0f,	6700.0f,1.0f};  	//速度环
int test_set_speed=10;

float PitchGyroPid[6] 				= {0.33f,		0.0f,			0.0f,		0.8f,			0.0f,	0.1f};  	//imu角度环
float PitchEncondePid[6] 			= {0.0f,		0.0f,			0.0f,		0.0f,		0.0f,		0.0f}; //编码器角度环
float PitchEncondePidSpeed[6] = {0.0f,	0.0f,		0.0f,	0.0f,	0.0f, 	0.0f}; //编码器速度环
//float PitchSpeedPid[6] 				= {0.2f,		0.03f,			0.8f,		12.0f,	8.0f, 0.0f};    //
float PitchSpeedPid[6] 				= {8900.0f,		320.0f,			0.0f,		3000.0f,	700.0f,		500.0f};    //

//原6020pitch轴电机参数
//float PitchGyroPid[5] 				= {10.0f,		5.0f,			0.0f,		20.0f,			1.0f};  	//imu角度环
//float PitchEncondePid[5] 			= {10.0f,		5.0f,			1.0f,		500.0f,		5000.0f}; //编码器角度环
//float PitchEncondePidSpeed[5] = {3000.0f,	100.0f,		10.0f,	30000.0f,	5000.0f}; //编码器速度环
//float PitchSpeedPid[5] 				= {380.0f,		220.0f,			150.0f,		15000.0f,	9500.0f};    //


//PID初始化	
static void YawPitch_PIDinit(void);
//云台电机模式选择
static void GIMBAL_Set_Mode(void);
//云台控制
static void GIMBAL_Set_Contorl(void);
//PID计算
static void GIMBAL_PID(void);
//IMU数据接收
static void GIMBAL_CALBACK_GET(void);

//速度PID
static void gimbal_motor_raw_pid(GIMBAL_t *gimbal_);
static void gimbal_motor_gyro_pid(GIMBAL_t *gimbal_);
static void gimbal_motor_encode_pid(GIMBAL_t *gimbal_);
int gimbal_imu_cnt=0;
int yu =0;//测试变量
float xuan=0;
/**
	* @brief       云台控制
	* @param				none
	* @retvel      none
*/
void Gimbal_Task(void)
{
	
	GIMBAL_CALBACK_GET();
	//模式选择
	GIMBAL_Set_Mode();
	//模式控制
	GIMBAL_Set_Contorl();
//	gimbal_y.gimbal_motor_mode=GIMBAL_MOTOR_RAW;
	//PID计算 
	GIMBAL_PID();
	//设定电流值
	gimbal_p.given_current=gimbal_p.given_current+850;  //gimbal_p.given_current+3;
	gimbal_y.given_current+=yu;
	//防止云台角度太低，齿轮脱齿，云台角度太高，齿轮崩碎板材
	if(gimbal_p.IMU_actual_angle<= -9)
	{
		gimbal_p.target_angle=-9;
	}
	if(gimbal_p.IMU_actual_angle>=50)
	{
		gimbal_p.target_angle=50;
	}
	
//	gimbal_p.given_current=0;
	
	canTX_pitch(gimbal_p.given_current);
	canTX_yaw(0,gimbal_y.given_current);
	
}
/*
	* @ brief       云台初始化函数
	* @ param				none
	* @ retvel      none
*/
void Gimbal_Init(void)
{
	
	YawPitch_PIDinit(); //PID初始化
	gimbal_set_mode=GIMBAL_ABSOLUTE_ANGLE;
	gimbal_y.IMU_actual_angle=0.0f;
	gimbal_y.IMU_actual_speed=0.0f;
	gimbal_y.CAN_actual_angle=0.0f;
	gimbal_y.CAN_actual_speed=0.0f;
	gimbal_y.target_angle=0.0f;
	gimbal_y.target_speed=0.0f;
	gimbal_y.add_angle=0.0f;
	gimbal_y.given_current=0;
	
	gimbal_y.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;//GIMBAL_MOTOR_RAW GIMBAL_MOTOR_GYRO
	
	gimbal_p.IMU_actual_angle=0.0f;
	gimbal_p.IMU_actual_speed=0.0f;
	gimbal_p.CAN_actual_speed=0.0f;
	
	gimbal_p.CAN_actual_angle=0.0f;
	gimbal_p.target_angle=0.0f; //(RC_PITCH_ANGLE_MAXX + RC_PITCH_ANGLE_MINN)/2.0f;  //初始时将pitch归至居中
	gimbal_p.target_speed=0.0f;
	gimbal_p.add_angle=0.0f;
	gimbal_p.given_current=0;
	
	gimbal_p.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;
}

/*
	* @ brief       yaw轴pitch轴初始化
	* @ param				none
	* @ retvel      none
*/
static void YawPitch_PIDinit(void)
{
	PID_Init(&gimbal_y.gimbal_raw_pid,YawSpeedPid);
	PID_Init(&gimbal_y.gimbal_gyro_pid,YawGyroPid);
	PID_Init(&gimbal_y.gimbal_enconde_pid,YawEncondePid);
	PID_Init(&gimbal_y.gimbal_enconde_pid_speed,YawEncondePidSpeed);
	
	PID_Init(&gimbal_p.gimbal_raw_pid,PitchSpeedPid);
	PID_Init(&gimbal_p.gimbal_gyro_pid,PitchGyroPid);
	PID_Init(&gimbal_p.gimbal_enconde_pid,PitchEncondePid);
	PID_Init(&gimbal_p.gimbal_enconde_pid_speed,PitchEncondePidSpeed);	
}

/*
	* @ brief       云台模式选择
	* @ param				none
	* @ retvel      none
*/
static void GIMBAL_Set_Mode(void)
{
    if (gimbal_set_mode == NULL)
    {
        return;
    }
	//电机模式选择
	if(gimbal_set_mode==GIMBAL_INIT)
	{
		gimbal_y.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;
		gimbal_p.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;
	}
	else if(gimbal_set_mode==GIMBAL_ZERO_FORCE)
	{
		gimbal_y.gimbal_motor_mode=GIMBAL_MOTOR_RAW;
		gimbal_p.gimbal_motor_mode=GIMBAL_MOTOR_RAW;
	}
	else if(gimbal_set_mode==GIMBAL_ABSOLUTE_ANGLE)
	{
		gimbal_y.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;
		gimbal_p.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;
	}
	else if(gimbal_set_mode==GIMBAL_TOP_ANGLE)
	{
		gimbal_y.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;
		gimbal_p.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;
	}
	else if(gimbal_set_mode==GIMBAL_RELATIVE_ANGLE)
	{
		gimbal_y.gimbal_motor_mode=GIMBAL_MOTOR_ENCONDE;
		gimbal_p.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;
	}
	else if(gimbal_set_mode==GIMBAL_CALI)
	{
		gimbal_y.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;
		gimbal_p.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;
	}
//	gimbal_y.gimbal_motor_mode=GIMBAL_MOTOR_RAW; //调试时选项
}

/*
	* @ brief       云台真正控制
	* @ param				none
	* @ retvel      none
*/
static void GIMBAL_Set_Contorl(void)
{	
	Vision_Task();
	
	if(vision_mode==VISION_OFF)
	{
		gimbal_y.target_angle+=yaw_angle;
		gimbal_p.target_angle+=pitch_angle;
	}
	else
	{
		gimbal_y.target_angle=yaw_angle;
		gimbal_p.target_angle=pitch_angle;
	}
	if(gimbal_set_mode!=GIMBAL_RELATIVE_ANGLE)
	{
		if(gimbal_y.target_angle>180)
		gimbal_y.target_angle -= 360;
	   if(gimbal_y.target_angle<-180)
		gimbal_y.target_angle += 360;
	}
	
}
/*
	* @ brief       云台反馈函数
	* @ param				none
	* @ retvel      none
*/
static void GIMBAL_CALBACK_GET(void)
{
    gimbal_y.CAN_actual_angle=(float)yaw_can_rx.angle; 
	gimbal_y.CAN_total_angle=(float)yaw_can_rx.turns*360.0f+gimbal_y.CAN_actual_angle;		//计算yaw的total_angle
	gimbal_y.CAN_actual_speed=(float)yaw_can_rx.speed;
	
	
  gimbal_y.IMU_actual_angle=INS_angle[0]/(2*3.141590f)*360.0f;
	gimbal_y.IMU_actual_speed=INS_gyro[2];
	
	
	gimbal_p.CAN_actual_angle=pitch_can_rx.angle/8191.0f*360.0f;
	gimbal_p.CAN_actual_speed=pitch_can_rx.speed;
	
  gimbal_p.IMU_actual_angle=-1.0f*INS_angle[1]/(2*3.141590f)*360.0f;
	gimbal_p.IMU_actual_speed= -1.0f*INS_gyro[0] + 0.0069813f;///(2*3.141590f)*360.0f +0.4f;
}


/*
	* @ brief       云台pid函数
	* @ param				none
	* @ retvel      none
*/
static void GIMBAL_PID(void)
{
	if(gimbal_y.gimbal_motor_mode==GIMBAL_MOTOR_RAW)
	{
		gimbal_motor_raw_pid(&gimbal_y);
	}
	if(gimbal_y.gimbal_motor_mode==GIMBAL_MOTOR_GYRO)
	{
		if(gimbal_y.target_angle-gimbal_y.IMU_actual_angle>183)
			gimbal_y.target_angle -= 360;
		if(gimbal_y.target_angle-gimbal_y.IMU_actual_angle<-180)
			gimbal_y.target_angle += 360;	 
		
			gimbal_motor_gyro_pid(&gimbal_y);
	}
	if(gimbal_y.gimbal_motor_mode==GIMBAL_MOTOR_ENCONDE)
	{
		gimbal_motor_encode_pid(&gimbal_y);
	}
	
	if(gimbal_p.gimbal_motor_mode==GIMBAL_MOTOR_RAW)
	{
		gimbal_motor_raw_pid(&gimbal_p);
	}
	if(gimbal_p.gimbal_motor_mode==GIMBAL_MOTOR_GYRO)
	{
		gimbal_motor_gyro_pid(&gimbal_p);
	}
	if(gimbal_p.gimbal_motor_mode==GIMBAL_MOTOR_ENCONDE)
	{
		gimbal_motor_encode_pid(&gimbal_p);
	}
}

 static void gimbal_motor_raw_pid(GIMBAL_t *gimbal_)
{
	PID_Calc(&gimbal_->gimbal_raw_pid,gimbal_->target_speed,gimbal_->IMU_actual_speed);
	
	gimbal_->given_current=gimbal_->gimbal_raw_pid.out;
}

static void gimbal_motor_gyro_pid(GIMBAL_t *gimbal_)
{
	PID_Calc(&gimbal_->gimbal_gyro_pid,gimbal_->target_angle,gimbal_->IMU_actual_angle);
	
	gimbal_->target_speed=gimbal_->gimbal_gyro_pid.out;
	
	
	PID_Calc(&gimbal_->gimbal_raw_pid,gimbal_->target_speed,gimbal_->IMU_actual_speed);
	
	gimbal_->given_current=gimbal_->gimbal_raw_pid.out;
}


static void gimbal_motor_encode_pid(GIMBAL_t *gimbal_)
{	
	//gimbal_->target_angle=temp_w;
	PID_Calc(&gimbal_->gimbal_enconde_pid,gimbal_->target_angle,gimbal_->CAN_total_angle);     //pitch的第三个参数为gimbal_->CAN_actual_angle

//	gimbal_->target_speed=temp_w;//gimbal_->gimbal_enconde_pid.out
	
	gimbal_->target_speed=gimbal_->gimbal_enconde_pid.out;
	
//    PID_Calc(&gimbal_->gimbal_enconde_pid_speed,gimbal_->target_speed,gimbal_->CAN_actual_speed);
//	
//	gimbal_->given_current=gimbal_->gimbal_enconde_pid_speed.out;
	PID_Calc(&gimbal_->gimbal_raw_pid,gimbal_->target_speed,gimbal_->IMU_actual_speed);
	
	gimbal_->given_current=gimbal_->gimbal_raw_pid.out;
}
	





