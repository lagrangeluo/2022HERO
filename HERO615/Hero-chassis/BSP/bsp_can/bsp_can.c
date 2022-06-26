/**
  ******************************************************************************
  * @file    Project/APP/bsp_can.c 
  * @author  NEUQRM
  * @version V1.0.0
  * @date    12.2021
  * @brief   该文件包含了CAN1对各电调的数据接收和CAN2实现对云台板子的数据接收
  ******************************************************************************
  * @attention
  ******************************************************************************
  ****************************(C) COPYRIGHT 2021 NEUQRM****************************

  ...........  革命尚未成功，同志仍需努力  ...........
*/

#include "bsp_can.h"

static int16_t* read_motor_data(uint8_t *rxdata,CAN_HandleTypeDef *hcan);
static void get_motor_data(MOTOR_t *motor, uint16_t angle, int16_t speed, int16_t current);
static void record_chassis_callback(MOTOR_t *motor, uint16_t angle, int16_t current);
static void record_yaw_callback(int16_t angle,int16_t speed);
uint8_t bsp_can_init(void)
{
	uint8_t status=0;
	CAN_FilterTypeDef canFilter;
	
	
	canFilter.FilterBank=1;    																//筛选器组1
	canFilter.FilterIdHigh=0;
	canFilter.FilterIdLow=0;
	canFilter.FilterMaskIdHigh=0;
	canFilter.FilterMaskIdLow=0;
	canFilter.FilterMode=CAN_FILTERMODE_IDMASK;  							//掩码模式
	canFilter.FilterActivation=CAN_FILTER_ENABLE;							//开启
	canFilter.FilterScale=CAN_FILTERSCALE_32BIT; 							//32位模式
	canFilter.FilterFIFOAssignment=CAN_FILTER_FIFO0; 					//链接到fifo0
	canFilter.SlaveStartFilterBank=14;												//can2筛选组起始编号
	
	status=HAL_CAN_ConfigFilter(&hcan1,&canFilter);					//配置过滤器
	
	canFilter.FilterBank=15;    															//筛选器组15
	status=HAL_CAN_ConfigFilter(&hcan2,&canFilter);					//配置过滤器
	
	/*离开初始模式*/
	HAL_CAN_Start(&hcan1);				
	HAL_CAN_Start(&hcan2);
	
	
	/*开中断*/
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);       //can1 接收fifo 0不为空中断
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);       //can2 接收fifo 0不为空中断
	return status;
}

int flag9=0;
uint8_t data[8];
uint8_t Can_Tx_Message(CAN_HandleTypeDef *hcan,uint8_t *mdata)
{
	int i;
	uint8_t status;
	CAN_TxHeaderTypeDef CAN_TxHeaderStruct;
	for(i=0;i<8;i++)
	data[i]=mdata[i];
	uint32_t  pTxMailbox;
	if(hcan==&hcan1)
	{
		flag9=1;
		CAN_TxHeaderStruct.StdId=0x200;
		CAN_TxHeaderStruct.ExtId=0;
		CAN_TxHeaderStruct.DLC=8;
		CAN_TxHeaderStruct.IDE=CAN_ID_STD;
		CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
		CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
	}
	else if(hcan==&hcan2)
	{
		CAN_TxHeaderStruct.StdId=0x007;
		CAN_TxHeaderStruct.ExtId=0;
		CAN_TxHeaderStruct.DLC=8;
		CAN_TxHeaderStruct.IDE=CAN_ID_STD;
		CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
		CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
	}
	status=HAL_CAN_AddTxMessage(hcan,&CAN_TxHeaderStruct,mdata,&pTxMailbox);
	return status;
}

int flag4=0;
float pich_angle;
int mode_now=1;
char M_DAta[5];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//CAN接收函数
{
	flag4=1;
	CAN_RxHeaderTypeDef CAN_RxHeaderStruct;
	uint8_t rxdata[8];
	int16_t speed,*gdata,current;
	float angle;
	if(hcan==&hcan1)//CAN1电调对应回调
	{
		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&CAN_RxHeaderStruct,rxdata);
		gdata=read_motor_data(rxdata,&hcan1);
		angle=gdata[0];
		speed=gdata[1];
		current=gdata[2];
		switch(CAN_RxHeaderStruct.StdId)
		{
			case CAN_3508Motor1_ID:
				get_motor_data(&chassis_motor1,angle,speed,current);
			break;
			case CAN_3508Motor2_ID:
				get_motor_data(&chassis_motor2,angle,speed,current);
			break;
			case CAN_3508Motor3_ID:
				get_motor_data(&chassis_motor3,angle,speed,current);
			break;
			case CAN_3508Motor4_ID:
				get_motor_data(&chassis_motor4,angle,speed,current);
			break;
			case CAN_GIMBAL_Y_ID:
				record_yaw_callback(angle,speed);
			break;
			case SHOOT_MOTOR_TRIGGER_ID:
				Trigger_Motor_Callback(&trigger,angle,speed);
			break;
			case LOOP_BACK_ID:
				get_motor_data(&chassis_motor1,angle,speed,current);
			default:break;
		}
	}
	else if(hcan==&hcan2)//此处是CAN2接收云台指令，ID与云台板子已对应
	{
		if(HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO0,&CAN_RxHeaderStruct,rxdata)!=HAL_OK)
			chassis_control_order.chassis_mode=CHASSIS_REMOTE_CLOSE;
		gdata=read_motor_data(rxdata,&hcan2);
		
		if(CAN_RxHeaderStruct.StdId==MODE_RECEIVE_ID)
		{
			chassis_control_order.chassis_mode=(uint8_t)gdata[0];
           chassis_control_order.last_chassis_mode=chassis_control_order.chassis_mode;
		}	
		if(CAN_RxHeaderStruct.StdId==GIMBAL_CONTROL_ID)
		{
			chassis_control_order.vx_set=(int16_t)((gdata[0]<<8)|gdata[1]); 
			chassis_control_order.vy_set=(int16_t)((gdata[2]<<8)|gdata[3]); 
			chassis_control_order.wz_set=(int16_t)((gdata[4]<<8)|gdata[5]);	
		}
		else if(CAN_RxHeaderStruct.StdId==TRIGGER_CONTROL_ID)
		{
			if(gdata[0]==1) 
			{
				  shoot_flag=1;
			    shoot_angle_clc();
				  gdata[0]=0;
			}
		}
		else if(CAN_RxHeaderStruct.StdId==YAW_CONTROL_ID)
		{
			gimbal_y.given_current=(int16_t)((gdata[2]<<8)|gdata[3]);//接收电流值
		}
		else if(CAN_RxHeaderStruct.StdId==UI_ID)
		{
			pich_angle=(int16_t)((gdata[0]<<8)|gdata[1]);
			pich_angle=(float)pich_angle/100.0f;
			mode_now=(int16_t)((gdata[2]<<8)|gdata[3]);
			mode_now=(float)mode_now;
			// 1 suidong 2 f 3 xiaotuoluo
//			if(mode_now==1) {M_DAta[0]='A';M_DAta[1]='A';}
//			if(mode_now==2) {M_DAta[0]='F';M_DAta[1]='F';}
//			if(mode_now==3) {M_DAta[0]='T';M_DAta[1]='T';}
		}
		
	}
	
}
int16_t adata[8];
static int16_t* read_motor_data(uint8_t *rxdata,CAN_HandleTypeDef *hcan)//分位读取数据函数
{
//	static int16_t adata[4];
	if(hcan==&hcan1)
	{
		adata[0]=(int16_t)((rxdata[0]<<8)|rxdata[1]);
		adata[1]=(int16_t)((rxdata[2]<<8)|rxdata[3]);
		adata[2]=(int16_t)((rxdata[4]<<8)|rxdata[5]);
		adata[3]=(int16_t)((rxdata[6]<<8)|rxdata[7]);
	}
	else if(hcan==&hcan2)//CAN2需全利用
	{
		adata[0]=(int16_t)(rxdata[0]);
		adata[1]=(int16_t)(rxdata[1]);
		adata[2]=(int16_t)(rxdata[2]);
		adata[3]=(int16_t)(rxdata[3]);
		adata[4]=(int16_t)(rxdata[4]);
		adata[5]=(int16_t)(rxdata[5]);
		adata[6]=(int16_t)(rxdata[6]);
		adata[7]=(int16_t)(rxdata[7]);
	}
	return adata;
}


static void get_motor_data(MOTOR_t *motor, uint16_t angle, int16_t speed, int16_t current)
{
	motor->last_angle = motor->actual_angle;
	motor->actual_angle = angle;
	motor->pid.position_loop.apid.actual_angle=angle;
	motor->actual_speed = 0.5*(speed + motor->last_speed);
	motor->pid.position_loop.vpid.actual_speed=motor->actual_speed;
	motor->pid.speed_loop.vpid.actual_speed=motor->actual_speed;
	//motor->actual_speed = KalmanFilter(speed,Q,R);
	motor->last_speed = speed;
	motor->actual_current = current;
	//motor1.temp = temp;
	if(motor->start_angle_flag==0)
	{
		motor->start_angle = angle;
		motor->start_angle_flag++;	//只在启动时记录一次初始角度
	}
	
	if(motor->actual_angle - motor->last_angle > 4096)
		motor->round_cnt--;
	else if (motor->actual_angle - motor->last_angle < -4096)
		motor->round_cnt++;
	motor->total_angle = motor->round_cnt * 8192 + motor->actual_angle;// - motor->start_angle;
}

static void record_chassis_callback(MOTOR_t *motor, uint16_t angle, int16_t current)
{
	motor->last_angle = motor->actual_angle;
	motor->actual_angle = (float)angle/8191.0f;
	motor->actual_angle*=360.0f;
	motor->actual_current = current;
	//motor1.temp = temp;
	if(motor->start_angle_flag==0)
	{
		motor->start_angle = (float)angle/8191.0f*360.0f;
		motor->start_angle_flag++;	//只在启动时记录一次初始角度
	}
	
}

static void record_yaw_callback(int16_t angle,int16_t speed)
{
		yaw_can_rx.lastangle = yaw_can_rx.angle;					
		yaw_can_rx.angle = (int16_t) angle;
		yaw_can_rx.speed = (int16_t) speed;
	    yaw_can_rx.turns=(yaw_can_rx.angle-yaw_can_rx.lastangle)<-4096?yaw_can_rx.turns+1:yaw_can_rx.turns;
		yaw_can_rx.turns=(yaw_can_rx.angle-yaw_can_rx.lastangle)>4096? yaw_can_rx.turns-1:yaw_can_rx.turns;	
		chassis_center.actual_angle=yaw_can_rx.angle*360.0f/8191.0f;
	
}

void send_gimbal_data_2(void)
{
	int16_t actual_vx,actual_vy;
	actual_vx=(int16_t)chassis_center.actual_angle;
	actual_vy=(int16_t)chassis_center.actual_speed;
	canTX_gimbal_2(actual_vx,actual_vy,shoot_flag);
}

void canTX_gimbal_2(int16_t vx,int16_t vy,int16_t shoot_flag_t)
{
	uint8_t data[8];
	uint8_t status;
	CAN_TxHeaderTypeDef CAN_TxHeaderStruct;
	uint32_t  pTxMailbox;
	
	CAN_TxHeaderStruct.StdId=CAN_GIMBAL_Y_ID;
	CAN_TxHeaderStruct.ExtId=0;
	CAN_TxHeaderStruct.DLC=8;
	CAN_TxHeaderStruct.IDE=CAN_ID_STD;
	CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
	CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
	
	data[0]=vx>>8;
	data[1]=vx&0xff;
	data[2]=vy>>8;
	data[3]=vy&0xff;
	data[4]=shoot_flag_t>>8;
	data[5]=shoot_flag_t&0xff;
	status=HAL_CAN_AddTxMessage(&hcan2,&CAN_TxHeaderStruct,data,&pTxMailbox);
}
