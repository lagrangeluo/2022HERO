#include "supercap.h"
#include "referee.h"


float supercap_volt;
int supercap_per;
uint8_t B='B';

/**
  * @breif         �������ݣ����ݲ�ͬ�Ĺ������޸������ϵ�Ƭ����������
  * @param[in]     none 
	* @param[out]    ����ͬ���ʵ��ַ�
  * @retval        none     
  */
	
int cap_cnt;
void supercap(void)
{
	cap_cnt++;
	static uint8_t send_data;
	uint16_t power_limit;
	get_chassis_power_limit(&power_limit);
	
	switch(power_limit)
	{
		case 40:
		{
			send_data='I';
			break;
		}
		case 45:
		{
			send_data='J';
			break;
		}
		case 50:
		{
			send_data='K';
			break;
		}
		case 55:
		{
			send_data='L';
			break;
		}
		case 60:
		{
			send_data='M';
			break;
		}
		case 80:
		{
			send_data='Q';
			break;
		}
		case 100:
		{
			send_data='U';
			break;
		}			
		default:
			send_data='M';
		break;
	}
	
	if(cap_cnt%2==0)
	HAL_UART_Transmit(&huart1, &send_data, 1, 20);
	
	if(cap_cnt%30==0)
//	HAL_UART_Transmit(&huart1, &B, 1, 20);

	if(cap_cnt>=100)
		cap_cnt=0;

	
}
