/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : gcubme2_beta version
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
union{
	uint32_t ui32t;
	float flt;
}u1,u2;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;

ETH_HandleTypeDef heth;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart6_rx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
uint8_t SysStatus = 0;  // system status
uint8_t ModStatus = 0;  // mode status
uint8_t test_mode = TRANS_RESIST_1;
uint8_t Step = 0;
uint8_t ACservoStatus = 0;	 // 0:stop | 1:start
uint8_t ActuatorStatus = 0;  // 0:stop | 1:forward | 2:backward | 3:ready
uint8_t direction = 0; // 1:forward | 0:backward
uint32_t EncoderCounter = 0;  // Encoder value of AC servo motor
uint32_t ActuatorCounter = 0;  // Encoder value of Actuator
uint8_t homming_flag=0;  // homing status

uint8_t usart2_buffer[10];  // Load cell receive buffer
uint8_t usart6_buffer[13];  // Torque sensor receive buffer
uint8_t usart5_buffer[13];  // Motor driver receive buffer
uint8_t usart5_data[13];
uint8_t temp_data[13];
uint8_t usart5_protocol[13];
uint8_t usart5_status=0;
int usart5_cnt=0;
uint8_t usart5_flag=0;
uint8_t usart5_ACK=0;
uint8_t usart5_REQ=0;
uint8_t abnormal = 0;

// Motor driver transmit protocol
// byte 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |10 |11 |12 |13 |
//     stx|len|id |cmd| Index |sub|     value     |crc|etx|
char usart2_data[7];  // data of Loadcell     0:+- | 1:x100  | 2:x10  | 3: x1 | 4: . | 5:x0.1 | 6:x0.01
char usart6_data[7];  // data of Torquesensor 0:+- | 1:x1000 | 2:x100 | 3:x10 | 4:x1 | 5: .   | 6:x0.1
char pulse_data[4];  // arranged data of Motor driver
int stroke1; // value of actuator1 stroke
int stroke2; // value of actuator2 stroke
char rx_data[10]; // data packet from TP
uint8_t tx_data[22];
uint8_t mycommand; // redefine the packet in master board

int torque = 0;
int pulse = 0;
int force = 0;  // value of load cell
int stroke = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
/*
int _write(int fd, char *str, int len)  //printf
{
	for(int i=0;i<len;i++)
	{
		HAL_UART_Transmit(&huart3, (uint8_t *)&str[i], 1, 0xFFFF);
	}
	return len;
}
*/
void CheckStatus(void);
void CheckSafety(void);
void CheckMode(void);
void Emergency(void);
void Rotate_Assist(void);
void Rotate_Resist(void);
void Translate_Assist(void);
void Translate_Resist(void);
void PushOff(void);
void wheel(void);
void Adjust_Height_Up(void);
void Adjust_Height_Down(void);

void Test(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)  // External Interrupt
{
  if(GPIO_Pin==GPIO_PIN_0)
    {
      SysStatus = 0b10000000;
      ModStatus = 0;
    }
}

void readLoadcell()
{
  if((usart2_buffer[0]==0xbf)&&(usart2_buffer[8]==0xf9))
  	{
  	  usart2_data[1]=(((~usart2_buffer[2])>>4)&0x0f);
  	  //data[1]=(((~buffer[2])>>4)&0x0f)+48;

  	  for(int i=2;i<=3;i++)
  	    {
  	      if(usart2_buffer[i]==0xfb)
  	      {
  	    	  usart2_data[i]=(((~usart2_buffer[i+1])>>4)&0x0f);
  		  //data[i]=(((~buffer[i+1])>>4)&0x0f)+48;
  		  }
  	      else
  	      {
  	    	  usart2_data[i]=(((~usart2_buffer[i+1])>>1)&0x0f);
  		  //data[i]=(((~buffer[i+1])>>1)&0x0f)+48;
  	      }
  	    }
  	  if(usart2_buffer[5]==0xa3)
  	    { usart2_data[4]='.'; }
  	  for(int j=5;j<=6;j++)
  	    {
  	      usart2_data[j]=(((~usart2_buffer[j+1])>>1)&0x0f);
  	      //data[j]=(((~buffer[j+1])>>1)&0x0f)+48;
  	    }

  	  force = (usart2_data[1]*10000)+(usart2_data[2]*1000)+
  	      (usart2_data[3]*100)+(usart2_data[5]*10)+(usart2_data[6]*1);

  	  if(usart2_buffer[1]==0x96)
  	    {
  	      usart2_data[0]='+';
  	    }
  	  else if(usart2_buffer[1]==0x56)
  	    {
  	      usart2_data[0]='-';
  	      force *= -1;
  	    }
  	}
}

void readTorque()
{
  if((usart6_buffer[0]==0xbf)&&(usart6_buffer[11]==0xf9))
  	{
  	  for(int i=1;i<=3;i++)
  	    {
  	      usart6_data[i]=(((~usart6_buffer[i+3])>>1)&0x0f);
  	    }
  	  if(usart6_buffer[7]==0xa3)
  	    { usart6_data[4]='.'; }
  	  usart6_data[5]=(((~usart6_buffer[8])>>1)&0x0f);
  	  usart6_data[6]=(((~usart6_buffer[9])>>1)&0x0f);

  	  torque = (usart6_data[1]*10000)+(usart6_data[2]*1000)+
  	      (usart6_data[3]*100)+(usart6_data[5]*10)+(usart6_data[6]*1);

  	  if(usart6_buffer[2]==0x96)
  	    {
  	      usart6_data[0]='+';
  	      torque *= -1;
  	    }
  	  else if(usart6_buffer[2]==0x56)
  	    {
  	      usart6_data[0]='-';
  	      //torque *= -1;
  	    }
  	}
}

/*
 received data packet come into channel (&huart4)
 tp sends shape of char[10] data
 */

uint8_t command()
{
	//rx_data[6]=  i;//loop counter  means height
	uint8_t sysmod = 0b00000000;

	for(int d=2; d<6; d++){//rxdata order d 2,3,4,5,6
	switch(d){
		case(2):
			switch(rx_data[2]){ // motor stauts
				case(0x31):
						sysmod = ROTATE;//rotat 1st High  0b10000000
					break;
				case(0x32):
						sysmod = TRANS;//trans 2nd High 0b01000000
					break;
				case(0x33):
						sysmod = PUSHOFF;//push & resist 3rd High 0b00100000
					break;
				case(0x34):
						sysmod = WHEELCYCLE;//wheel & resist 4th High 0b00010000
					break;
			}
		case(3):
			switch(rx_data[3]){//mode status
			case(0x41):
					sysmod |= MOE;//Activemode 6th High 0b00000100
				break;
			case(0x52):
					sysmod &= ~(0x04);//Resistmode 6th Low 0b00010000
				break;
			case(0x30):
					sysmod &= ~(0x04);//N/A mode 6th low push and wheel has just resist mode 0b00010000
				break;
			}
		case(4):
			switch(rx_data[4]){//speed
			case(0x00):
					sysmod &= ~(0x03);// using last two bit 00 -> 0b00000011
				break;
			case(0x61): //make 01
					sysmod &= ~(0x02);//using last two bit 01 -> 0b00000010
					sysmod |= (0x01); //->0b00000001
				break;
			case(0x62)://make 10
					sysmod &= ~(0x01);//using last two bit 10 -> 0b00000001
					sysmod |= (0x02);//->0b00000010
				break;
			case(0x63)://make 11
					sysmod |= 0x03;//using last two bit 11 ->0b00000011
				break;
			}
		case(5):
			switch(rx_data[5]){//activate up down mode
			case(0x55):
					sysmod = 0x00;
					sysmod |= 0x08;//activate up down mode
					sysmod |= 0x04;//up drive
				break;
			case(0x44):
					sysmod = 0x00;
					sysmod |= 0x08;//activate up down mode
					sysmod &= ~(0x04); //down drive
				break;
			case(0x53):
					sysmod = 0x00;
					sysmod &= ~(0x08);//deactivate up down mode
				break;
			}
		}
	}
	return sysmod;
}

void readMotorDriver()
{
	int value=0;
	static int fst_num=0;
	//static uint8_t err_point=0;

	if((usart5_buffer[0]==STX)&&(usart5_buffer[12]==ETX))
	{
		abnormal=0;
		for(int i=0;i<13;i++)
		{
			usart5_data[i]=usart5_buffer[i];
		}
	}
	else
	{
		abnormal=1;
		//__HAL_UART_CLEAR_OREFLAG(&huart5);
		//HAL_UART_DMAStop(&huart5);
		//HAL_UART_Receive_DMA(&huart5, (uint8_t *)usart5_buffer, 13);
		for(int i=0;i<13;i++)
		{
			if((usart5_buffer[i]==0x03)&&(usart5_buffer[i+1]==0x02))
			{
				fst_num=i+1;
			}

			if(i>=fst_num)
			{
				temp_data[i-fst_num]=usart5_buffer[i];
			}
			else if(i<fst_num)
			{
				usart5_data[13-fst_num+i]=usart5_buffer[i];
				if(i<13-fst_num)
				{
					usart5_data[i]=temp_data[i];
				}
			}
		}
	}

	if((usart5_data[3]&0xf0)==RESPONSE_READ)
	{
		value=((int)usart5_data[7]<<0)|
				((int)usart5_data[8]<<8)|
				((int)usart5_data[9]<<16)|
				((int)usart5_data[10]<<24);
		if(usart5_data[6]==CH1)
		{
			stroke1=value;
			usart5_status=POS_CH1;
		}
		else if(usart5_data[6]==CH2)
		{
			stroke2=value;
			usart5_status=POS_CH2;
		}
		usart5_ACK=1;
		usart5_REQ=0;
	}
	else if((usart5_data[3]&0xf0)==RESPONSE_WRITE)
	{
		if(usart5_data[6]==CH1)
		{
			usart5_status=CUR_CH1;
		}
		else if(usart5_data[6]==CH2)
		{
			usart5_status=CUR_CH2;
		}
		usart5_ACK=1;
		usart5_REQ=0;
	}


/*
//Serial text protocol :
//Received data of Motor driver is like
//"mp=1000,1000\r\n"

  for(int i=0; i<12; i++)
    {
      if((usart5_buffer[i]=='m')&&(usart5_buffer[i+1]=='p')&&(usart5_buffer[i+2]=='='))
      {
    	  int j=0;
    	  while(j<4)
    	  {
    		  if(usart5_buffer[i+j+3]==',')
    		  {
    			  break;
    		  }
    		  else
    		  {
    			  pulse_data[j]=usart5_buffer[i+j+3];
    			  j++;
    		  }
    	  }
      }
    }
  stroke = atoi(pulse_data);
*/
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{ //RX callback function
  if(huart->Instance==USART2)
    {
      HAL_UART_Receive_DMA(&huart2, (uint8_t *)usart2_buffer, 10);
      readLoadcell();
    }
  if(huart->Instance == USART3) //matching Instance of huart  with UART3  channel
	{
	  HAL_UART_Receive_DMA(&huart3, (uint8_t *)rx_data, 10);// from where /which data address/ data length
      mycommand = command();
    }
  if(huart->Instance == UART4) //matching Instance of huart  with UART4  channel
    {
      HAL_UART_Receive_DMA(&huart4, (uint8_t *)rx_data, 10);// from where /which data address/ data length
      mycommand = command();
    }
  if(huart->Instance==UART5)
    {
/*
	  printf("ACK : ");
	  for(int i=0;i<13;i++)
	  {
		  printf("%x  ",usart5_buffer[i]);
	  }
	  printf("\r\n");
*/
	  readMotorDriver();
	  HAL_UART_Receive_DMA(&huart5, (uint8_t *)usart5_buffer, 13);

    }
  if(huart->Instance==USART6)
    {
      HAL_UART_Receive_DMA(&huart6, (uint8_t *)usart6_buffer, 13);
      readTorque();
    }
}

char CheckSum(uint8_t *msg, int start, int end)
{
	char cs=0;
	for(int i=start;i<=end;i++)
	{
		cs += msg[i];
	}
	return cs;
}

void MultiPosition(void)
{
	//uint8_t text_packet[5]={0x6d, 0x70, 0x30, 0x0d, 0x0a};
	//uint8_t text_packet[4]={0x6d, 0x70, 0x0d, 0x0a};
	//HAL_UART_Transmit(&huart5, (uint8_t *)text_packet, 4, 100);
/*
	for(int i=0;i<5;i++)
		{
			HAL_UART_Transmit(&huart5, (uint8_t *)&text_packet[i], 1, 0xFFFF);
		}
*/
	/*
	text_packet[0]=0x6d;
	text_packet[1]=0x70;
	text_packet[2]=0x0d;
	text_packet[3]=0x0a;

	HAL_UART_Transmit(&huart5, (uint8_t*)text_packet, 4, 10);
	*/
	//HAL_UART_Transmit(&huart5, (uint8_t *)"mp", 2, 0xffff);
	//HAL_UART_Transmit(&huart5, (uint8_t *)"\r\n", 2, 0xffff);
	//HAL_UART_Receive_DMA(&huart5, (uint8_t *)usart5_buffer, 31);
	HAL_UART_Transmit(&huart5, (uint8_t *)"mpc=0,0\r\n", 9, 0xffff);
}
void RotateCommand(int pos)
{
	HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);
	HAL_Delay(pos);
	HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_3);
	HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_4);
	HAL_Delay(10-pos);
}

void CheckPosition()
{
	/*
	uint8_t channel=0;
	static uint8_t toggle=0;
	if(toggle==0)
	{
		channel=CH1;
	}
	else if(toggle==1)
	{
		channel=CH2;
	}
*/
	if((usart5_REQ==0)&&(usart5_flag==0))
	{
		usart5_protocol[0]=STX;
		usart5_protocol[1]=DTL;
		usart5_protocol[2]=1;
		usart5_protocol[3]=REQUEST_READ+INT32;
		usart5_protocol[4]=POSITION;
		usart5_protocol[5]=0;
		usart5_protocol[6]=CH1;
		usart5_protocol[7]=0;
		usart5_protocol[8]=0;
		usart5_protocol[9]=0;
		usart5_protocol[10]=0;
		usart5_protocol[11]=CheckSum(usart5_protocol,2,10);
		usart5_protocol[12]=ETX;
		/*
		printf("REQ : ");
		for(int i=0;i<13;i++)
		{
		  printf("%x  ",usart5_protocol[i]);
		}
		printf("\r\n");
		*/
		HAL_UART_Transmit(&huart5, (uint8_t*)usart5_protocol, 13, 20);
		usart5_REQ=1;
		/*
		printf("REQ : ");
		for(int i=0;i<13;i++)
		{
			printf("%x  ",usart5_protocol[i]);
		}
		printf("\r\n");
		*/
		HAL_Delay(200);
	}
	usart5_cnt++;

	if((usart5_status==POS_CH1)||(usart5_cnt>20))
	{
		//toggle^=1;
		//usart5_status=0;
		usart5_flag=0;
		usart5_REQ=0;
		usart5_cnt=0;
	}

	//static int cnt=0;
	//if((cnt==0)||(cnt==5))
	/*
	if((usart5_status==0)&&(usart5_REQ==0))
	{
		usart5_protocol[0]=STX;
		usart5_protocol[1]=13;  // data length
		usart5_protocol[2]=1;
		usart5_protocol[3]=REQUEST_READ+INT32;
		usart5_protocol[4]=POSITION;
		usart5_protocol[5]=0;
		usart5_protocol[6]=CH1;
		usart5_protocol[7]=0;
		usart5_protocol[8]=0;
		usart5_protocol[9]=0;
		usart5_protocol[10]=0;
		usart5_protocol[11]=CheckSum(usart5_protocol,2,10);
		usart5_protocol[12]=ETX;
		HAL_UART_Transmit(&huart5, (uint8_t*)usart5_protocol, 13, 20);
		usart5_flag=0;
		usart5_REQ=1;
		HAL_Delay(100);
	}
	if((usart5_status==POS_CH1)&&(usart5_REQ==0))
	{
		usart5_protocol[0]=STX;
		usart5_protocol[1]=13;  // data length
		usart5_protocol[2]=1;
		usart5_protocol[3]=REQUEST_READ+INT32;
		usart5_protocol[4]=POSITION;
		usart5_protocol[5]=0;
		usart5_protocol[6]=CH2;
		usart5_protocol[7]=0;
		usart5_protocol[8]=0;
		usart5_protocol[9]=0;
		usart5_protocol[10]=0;
		usart5_protocol[11]=CheckSum(usart5_protocol,2,10);
		usart5_protocol[12]=ETX;
		HAL_UART_Transmit(&huart5, (uint8_t*)usart5_protocol, 13, 100);
		usart5_REQ=1;
		HAL_Delay(100);
	}
	usart5_cnt++;
	if(usart5_status==POS_CH2)
	{
		usart5_status=0;
		usart5_flag=1;
		usart5_REQ=0;
		usart5_cnt=0;
	}

	if(usart5_cnt>100)
	{
		usart5_REQ=0;
		usart5_cnt=0;
	}
*/
	//cnt++;
	//if(cnt>=10)
	//{
	//	cnt=0;
	//}
	//HAL_UART_Receive_DMA(&huart5, (uint8_t *)usart5_buffer, 13);
	//HAL_Delay(100);
}

void PositionCommand(uint8_t channel, int value)
{
	usart5_protocol[0]=STX;
	usart5_protocol[1]=13;
	usart5_protocol[2]=1;
	usart5_protocol[3]=REQUEST_WRITE+INT32;
	usart5_protocol[4]=POSITION_CMD;
	usart5_protocol[5]=0;
	usart5_protocol[6]=channel;
	usart5_protocol[7]=(uint8_t)((value>>0)&0xff);
	usart5_protocol[8]=(uint8_t)((value>>8)&0xff);
	usart5_protocol[9]=(uint8_t)((value>>16)&0xff);
	usart5_protocol[10]=(uint8_t)((value>>24)&0xff);
	usart5_protocol[11]=CheckSum(usart5_protocol,2,10);
	usart5_protocol[12]=ETX;
	HAL_UART_Transmit(&huart5, (uint8_t*)usart5_protocol, 13, 20);
	//HAL_UART_Receive_DMA(&huart5, (uint8_t *)usart5_buffer, 13);
	HAL_Delay(20);
}

void CurrentCommand(float value)
{
	u1.flt=value;

	if((usart5_status!=CUR_CH1)&&(usart5_REQ==0))
	{
		usart5_flag=1;
		usart5_protocol[0]=STX;
		usart5_protocol[1]=DTL;
		usart5_protocol[2]=1;
		usart5_protocol[3]=REQUEST_WRITE+FLOAT;
		usart5_protocol[4]=CURRENT_CMD;
		usart5_protocol[5]=0;
		usart5_protocol[6]=CH1;
		usart5_protocol[7]=(u1.ui32t>>0)&0xff;  //(uint8_t)(*((uint32_t*)&value)>>0);
		usart5_protocol[8]=(u1.ui32t>>8)&0xff;//(uint8_t)((*((uint32_t*)&value)>>8)&0xff);
		usart5_protocol[9]=(u1.ui32t>>16)&0xff;//(uint8_t)((*((uint32_t*)&value)>>16)&0xff);
		usart5_protocol[10]=(u1.ui32t>>24)&0xff;//(uint8_t)((*((uint32_t*)&value)>>24)&0xff);
		usart5_protocol[11]=CheckSum(usart5_protocol,2,10);
		usart5_protocol[12]=ETX;
		HAL_UART_Transmit(&huart5, (uint8_t*)usart5_protocol, 13, 100);
		usart5_REQ=1;
		HAL_Delay(50);
	}
	if((usart5_status!=CUR_CH2)&&(usart5_REQ==0))
	{
		usart5_protocol[0]=STX;
		usart5_protocol[1]=DTL;
		usart5_protocol[2]=1;
		usart5_protocol[3]=REQUEST_WRITE+FLOAT;
		usart5_protocol[4]=CURRENT_CMD;
		usart5_protocol[5]=0;
		usart5_protocol[6]=CH2;
		usart5_protocol[7]=(u1.ui32t>>0)&0xff;  //(uint8_t)(*((uint32_t*)&value)>>0);
		usart5_protocol[8]=(u1.ui32t>>8)&0xff;//(uint8_t)((*((uint32_t*)&value)>>8)&0xff);
		usart5_protocol[9]=(u1.ui32t>>16)&0xff;//(uint8_t)((*((uint32_t*)&value)>>16)&0xff);
		usart5_protocol[10]=(u1.ui32t>>24)&0xff;//(uint8_t)((*((uint32_t*)&value)>>24)&0xff);
		usart5_protocol[11]=CheckSum(usart5_protocol,2,10);
		usart5_protocol[12]=ETX;
		HAL_UART_Transmit(&huart5, (uint8_t*)usart5_protocol, 13, 100);
		usart5_REQ=1;
		HAL_Delay(200);
	}
	/*
	if((usart5_status==CUR_CH2)&&(usart5_REQ==0))
	{
		usart5_protocol[0]=STX;
		usart5_protocol[1]=DTL;
		usart5_protocol[2]=1;
		usart5_protocol[3]=REQUEST_READ+INT32;
		usart5_protocol[4]=POSITION;
		usart5_protocol[5]=0;
		usart5_protocol[6]=CH1;
		usart5_protocol[7]=0;
		usart5_protocol[8]=0;
		usart5_protocol[9]=0;
		usart5_protocol[10]=0;
		usart5_protocol[11]=CheckSum(usart5_protocol,2,10);
		usart5_protocol[12]=ETX;
		printf("REQ : ");
		for(int i=0;i<13;i++)
		{
		  printf("%x  ",usart5_protocol[i]);
		}
		printf("\r\n");
		HAL_UART_Transmit(&huart5, (uint8_t*)usart5_protocol, 13, 20);
		usart5_REQ=1;
		HAL_Delay(100);
	}
	*/
	usart5_cnt++;
	if((usart5_status==CUR_CH2)||(usart5_cnt>20))
	{
		//usart5_status=0;
		usart5_flag=0;
		usart5_REQ=0;
		usart5_cnt=0;
	}


	//HAL_UART_Receive_DMA(&huart5, (uint8_t *)usart5_buffer, 13);
}

void SendData(int torque, int pulse, int force, int stroke)
{
  tx_data[0] = 0x02;
  tx_data[1] = 0x10;
  tx_data[2] = (uint8_t)((torque>>24)&0xff);
  tx_data[3] = (uint8_t)((torque>>16)&0xff);
  tx_data[4] = (uint8_t)((torque>>8)&0xff);
  tx_data[5] = (uint8_t)((torque>>0)&0xff);
  tx_data[6] = (uint8_t)((pulse>>24)&0xff);
  tx_data[7] = (uint8_t)((pulse>>16)&0xff);
  tx_data[8] = (uint8_t)((pulse>>8)&0xff);
  tx_data[9] = (uint8_t)((pulse>>0)&0xff);
  tx_data[10] = (uint8_t)((force>>24)&0xff);
  tx_data[11] = (uint8_t)((force>>16)&0xff);
  tx_data[12] = (uint8_t)((force>>8)&0xff);
  tx_data[13] = (uint8_t)((force>>0)&0xff);
  tx_data[14] = (uint8_t)((stroke>>24)&0xff);
  tx_data[15] = (uint8_t)((stroke>>16)&0xff);
  tx_data[16] = (uint8_t)((stroke>>8)&0xff);
  tx_data[17] = (uint8_t)((stroke>>0)&0xff);
  tx_data[18] = CheckSum(tx_data, 2, 17);
  tx_data[19] = 0x03;
  tx_data[20] = 0x0a;
  tx_data[21] = 0x0d;

  HAL_UART_Transmit(&huart3, (uint8_t *)tx_data, 22, 100);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_DAC_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_DMA(&huart2, (uint8_t *)usart2_buffer, 10);
  HAL_UART_Receive_DMA(&huart3, (uint8_t *)rx_data, 10);
  HAL_UART_Receive_DMA(&huart4, (uint8_t *)rx_data, 10);
  HAL_UART_Receive_DMA(&huart5, (uint8_t *)usart5_buffer, 13);
  HAL_UART_Receive_DMA(&huart6, (uint8_t *)usart6_buffer, 13);

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1|TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  CheckStatus();
	  CheckSafety();
	  CheckMode();
	  SendData(torque, pulse, force, stroke1);
	  //printf("force=%d, CH1=%d, CH2=%d\r\n", force, stroke1, stroke2);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization 
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config 
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   uint8_t MACAddr[6] ;

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  heth.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
  heth.Init.PhyAddress = LAN8742A_PHY_ADDRESS;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.RxMode = ETH_RXPOLLING_MODE;
  heth.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
  heth.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;

  /* USER CODE BEGIN MACADDRESS */
    
  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 3360-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000000000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PF0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PG1 PG2 PG3 PG4 
                           USB_PowerSwitchOn_Pin PG9 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4 
                          |USB_PowerSwitchOn_Pin|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD1_Pin */
  GPIO_InitStruct.Pin = LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void CheckStatus(void)
{
  if(SysStatus&MASK_EMERGENCY)
    {
      Emergency();
    }
  else
    {
	  HAL_UART_Receive_DMA(&huart3, (uint8_t *)rx_data, 10);
      HAL_UART_Receive_DMA(&huart4, (uint8_t *)rx_data, 10);
      ModStatus=mycommand; // TP mycommand is global variable and mycommand=command()
      //SendData(torque, pulse, force, stroke);
      //ModStatus=test_mode;
    }
}

void CheckSafety(void)
{
	if(ModStatus&(MASK_ROTATE|MASK_PUSHOFF|MASK_WHEEL))
	{
		if(SysStatus&MASK_LNA)
		{
			CheckPosition();
			if(stroke1>100)
			{
				CurrentCommand(-1.1);
			}
			else
			{
				CurrentCommand(0.0);
				ActuatorStatus = 0;
				direction = 0;
				SysStatus&=(~MASK_LNA);
			}
		}
		else
		{
			if(!(SysStatus&MASK_SRV))
			{
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);
				SysStatus|=MASK_SRV;
			}
			if(!(SysStatus&MASK_BRK))
			{
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET);
				SysStatus|=MASK_BRK;
			}
			if(!(SysStatus&MASK_DAC))
			{
				HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
				SysStatus|=MASK_DAC;
			}
			if(!(SysStatus&MASK_ENC))
			{
				HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1|TIM_CHANNEL_2);
				SysStatus|=MASK_ENC;
			}
		}
	}
	else if(ModStatus&MASK_TRANSLATE)
	{
		SysStatus|=MASK_LNA;
		if(SysStatus&MASK_SRV)
		{
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);
			SysStatus&=(~MASK_SRV);
		}
		if(SysStatus&MASK_BRK)
		{
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET);
	      	SysStatus&=(~MASK_BRK);
		}
		if(SysStatus&MASK_CL)
		{
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET);
	    	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_SET);
	        SysStatus&=(~MASK_CL);
		}
		if(SysStatus&MASK_MBRK)
		{
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);
			SysStatus&=(~MASK_MBRK);
		}
		if(SysStatus&MASK_DAC)
		{
			HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);
	    	SysStatus&=(~MASK_DAC);
		}
		if(SysStatus&MASK_ENC)
		{
			HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_1|TIM_CHANNEL_2);
			EncoderCounter = 0;
			SysStatus&=(~MASK_ENC);
		}
	}

	else //if(!(ModStatus&(MASK_ROTATE|MASK_PUSHOFF|MASK_WHEEL)))
	{
		if(SysStatus&MASK_LNA)
		{
			CheckPosition();
			if(stroke1>100)
			{
				CurrentCommand(-1.1);
			}
			else
			{
				CurrentCommand(0.0);
				ActuatorStatus = 0;
				direction = 0;
				SysStatus&=(~MASK_LNA);
			}
		}
		if(SysStatus&MASK_SRV)
		{
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);
	    	SysStatus&=(~MASK_SRV);
		}
		if(SysStatus&MASK_BRK)
		{
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET);
	      	SysStatus&=(~MASK_BRK);
		}
		if(SysStatus&MASK_CL)
		{
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET);
	    	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_SET);
	    	SysStatus&=(~MASK_CL);
		}
		if(SysStatus&MASK_MBRK)
		{
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);
			SysStatus&=(~MASK_MBRK);
		}
		if(SysStatus&MASK_DAC)
		{
			HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);
			SysStatus&=(~MASK_DAC);
		}
		if(SysStatus&MASK_ENC)
		{
			HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_1|TIM_CHANNEL_2);
			EncoderCounter = 0;
			SysStatus&=(~MASK_ENC);
		}
		ACservoStatus = 0;
	}
}

void CheckMode(void)
{
  Step = ModStatus&MASK_STEP;
  if(ModStatus&MASK_ROTATE)
    {
      if(ModStatus&MASK_ASSIST)
      {
    	  Rotate_Assist();
      }
      else if(!(ModStatus&MASK_ASSIST))
      {
    	  Rotate_Resist();
      }
    }
  else if(ModStatus&MASK_TRANSLATE)
    {
      if(ModStatus&MASK_ASSIST)
      	{
      	  Translate_Assist();
      	}
      else if(!(ModStatus&MASK_ASSIST))
      	{
      	  Translate_Resist();
      	}
    }
  else if(ModStatus&MASK_PUSHOFF)
    {
      PushOff();
    }
  else if(ModStatus&MASK_WHEEL)
    {
      wheel();
    }
  else if(ModStatus&MASK_HEIGHT)
    {
      if(ModStatus&MASK_ASSIST)
      	{
    	  Adjust_Height_Up();
      	}
      else if(!(ModStatus&MASK_ASSIST))
      	{
          Adjust_Height_Down();
      	}
    }
  else
    {
       //Test();
    }
}

void Test(void)
{
	/*
	CheckPosition();
	if(stroke1>=TRANS_POINT_A)
	{
		CurrentCommand(-1.0);
	}
	else if(stroke1<=TRANS_POINT_B)
	{
		CurrentCommand(1.0);
	}
	*/
}

void Emergency(void)
{
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_SET);
  HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);
  HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_1|TIM_CHANNEL_2);
  //HAL_UART_Transmit(&huart5, (uint8_t *)"cc=0\r\n", 6, 100);
  CheckPosition();
  CurrentCommand(0.0);
  ActuatorStatus=0;

  if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0)==0)
    {
      SysStatus = 0;
      ModStatus = 0;
    }
}

void Rotate_Assist(void)
{
	//static uint32_t OldPos = 0;
	static uint8_t AssistTorque = 0;
	static int torque_lmt = 0;

	//HAL_UART_Transmit(&huart2, (uint8_t *)"0R", 2, 100);
	//HAL_UART_Receive_DMA(&huart2, (uint8_t *)usart2_buffer, 10);

	switch(Step)
	{
	  case 0:
		  AssistTorque = ASSIST_TORQUE_DEFAULT;
		break;
	  case 1:
		  AssistTorque = ASSIST_TORQUE_STEP1;
		break;
	  case 2:
		  AssistTorque = ASSIST_TORQUE_STEP2;
		  break;
	  case 3:
		  AssistTorque = ASSIST_TORQUE_STEP3;
		  break;
	}
	torque_lmt = AssistTorque*(-3);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, AssistTorque);
	EncoderCounter = TIM2->CNT;
	pulse = (int)EncoderCounter;

	//RotateCommand(2);

	if(ACservoStatus==0)
	{
		if(torque>=20)
		{
			RotateCommand(1);
			ACservoStatus = 1;
		}
		else
		{
			HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_1);
			HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_2);
			HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_3);
			HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_4);
			HAL_Delay(10);
			ACservoStatus = 0;
		}
	}
	else if(ACservoStatus==1)
	{
	    if(torque<torque_lmt)
	    {
	    	HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_1);
	    	HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_2);
	    	HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_3);
	    	HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_4);
	    	HAL_Delay(10);
	    	ACservoStatus = 0;
	    }
	    else if((torque>=torque_lmt)&&(torque<20))
	    {
	    	RotateCommand(2);
	    	ACservoStatus = 1;
	    }
	    else if(torque>=20)
	    {
	    	uint8_t pcmd = (uint8_t)((130+torque)/50);
	    	if(pcmd>9){pcmd=9;}
	    	RotateCommand(pcmd);
	    	ACservoStatus = 1;
	    }
	  }

	/*
	if((EncoderCounter-OldPos)>=15)
	{
		RotateCommand(1);
	}
	else
	{
		HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_2);
		HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_3);
		HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_4);
		HAL_Delay(2);
	}
	*/
	//SendData(torque, pulse, force, stroke);
	//printf("%d %d\r\n", EncoderCounter, EncoderCounter-OldPos);
	//OldPos = EncoderCounter;
}

void Rotate_Resist(void)
{
  //static uint32_t OldPos = 0;
  static uint8_t ResistTorque = 0;

  switch(Step)
  {
    case 0:
    	ResistTorque = RESIST_TORQUE_DEFAULT;
        break;
    case 1:
    	ResistTorque = RESIST_TORQUE_STEP1;
        break;
    case 2:
    	ResistTorque = RESIST_TORQUE_STEP2;
        break;
    case 3:
    	ResistTorque = RESIST_TORQUE_STEP3;

        if(!(SysStatus&MASK_MBRK))
        {
        	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET);
            SysStatus|=MASK_MBRK;
        }

        break;
    }
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, ResistTorque);

  EncoderCounter = TIM2->CNT;
  pulse = (int)EncoderCounter;

  if(!(SysStatus&MASK_CL))
    {
	  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_RESET);
      SysStatus|=MASK_CL;
    }
  //SendData(torque, pulse, force, stroke);
  HAL_Delay(10);
  //printf("%d\t%d\r\n", EncoderCounter, force);
  //OldPos = EncoderCounter;
}

void Translate_Assist(void)
{
  static int AssistForce = 0;

  //HAL_UART_Transmit(&huart2, (uint8_t *)"0R", 2, 100);
  HAL_UART_Receive_DMA(&huart2, (uint8_t *)usart2_buffer, 10);

  //HAL_UART_Transmit(&huart5, (uint8_t *)"mp\r\n", 4, 100);
  //ActuatorCounter = TIM2->CNT;
  CheckPosition();

  switch(Step)
    {
      case 0:
    	  AssistForce = ASSIST_FORCE_DEFAULT;
    	  break;
      case 1:
    	  AssistForce = ASSIST_FORCE_STEP1;
          break;
      case 2:
    	  AssistForce = ASSIST_FORCE_STEP2;
          break;
      case 3:
    	  AssistForce = ASSIST_FORCE_STEP3;
        break;
    }

  if(force > AssistForce)
    {
      direction = 2;  // forward
    }
  else if(force < (AssistForce*(-1)))
    {
      direction = 1;  // backward
    }

  if(abnormal)
  {
	  __HAL_UART_CLEAR_OREFLAG(&huart5);
	  HAL_UART_DMAStop(&huart5);
	  HAL_UART_Receive_DMA(&huart5, (uint8_t *)usart5_buffer, 13);

	  //ActuatorStatus=0;
	  if(ActuatorStatus==0)
	  {
		  //CurrentCommand(0.0);
		  ActuatorStatus = 3;
	  }
	  else if(ActuatorStatus==1)
	  {
	      switch(Step)
	      {
	      	case 0:
	      		CurrentCommand(0.0);
	      		break;
	      	case 1:
	      		CurrentCommand(1.1);
	      		break;
	        case 2:
	        	CurrentCommand(1.0);
	        	break;
	        case 3:
	        	CurrentCommand(0.9);
	        	break;
	        }
	      ActuatorStatus = 3;
	  }
	  else if(ActuatorStatus==2)
	  {
	      switch(Step)
	      {
	        case 0:
	        	CurrentCommand(0.0);
	        	break;
	        case 1:
	        	CurrentCommand(-1.1);
	        	break;
	        case 2:
	        	CurrentCommand(-1.0);
	        	break;
	        case 3:
	        	CurrentCommand(-0.9);
	        	break;
	      }
	      ActuatorStatus = 3;
	  }
	  else if(ActuatorStatus==3)
	  {
		  if(direction==1)
		  {
			  ActuatorStatus = 1;
		  }
		  else if(direction==2)
		  {
			  ActuatorStatus = 2;
		  }
	  }
  }
  else
  {
	  if(ActuatorStatus==0)
	  {
		  CurrentCommand(0.0);
		  ActuatorStatus = 3;
	  }
	  else if(ActuatorStatus==1)
	  {
		  if(stroke1>=TRANS_POINT_A)
		  {
			  ActuatorStatus = 3;
		  }
		  else
		  {
			  switch(Step)
			  {
			  	  case 0:
			  		  CurrentCommand(0.0);
			  		  break;
			  	  case 1:
			  		  CurrentCommand(1.1);
			  		  break;
			  	  case 2:
			  		  CurrentCommand(1.0);
			  		  break;
			  	  case 3:
			  		  CurrentCommand(0.9);
			  		  break;
			  }
		  }
	  }
	  else if(ActuatorStatus==2)
	  {
		  if(stroke1<=TRANS_POINT_B)
		  {
			  ActuatorStatus = 3;
		  }
		  else
		  {
			  switch(Step)
			  {
			  	  case 0:
			  		  CurrentCommand(0.0);
			  		  break;
			  	  case 1:
			  		  CurrentCommand(-1.1);
			  		  break;
			  	  case 2:
			  		  CurrentCommand(-1.0);
			  		  break;
			  	  case 3:
			  		  CurrentCommand(-0.9);
			  		  break;
			  }
		  }
	  }
	  else if(ActuatorStatus==3)
	  {
		  if(direction==1)
		  {
			  ActuatorStatus = 1;
		  }
		  else if(direction==2)
		  {
			  ActuatorStatus = 2;
		  }

	  }
	  /*
	  if(ActuatorStatus==0)
	    {
		  if(stroke1<=TRANS_POINT_B)//||(stroke2<=TRANS_POINT_B))
		  {
			  if(direction==1)
			  {
				  ActuatorStatus = 1;
			  }
		  }
	    }
	  else if(ActuatorStatus==1)
	  {
		  switch(Step)
		  {
		  	  case 0:
		  		  CurrentCommand(0.0);
		  		  break;
		  	  case 1:
		  		  CurrentCommand(1.1);
		  		  break;
		  	  case 2:
		  		  CurrentCommand(1.0);
		  		  break;
		  	  case 3:
		  		  CurrentCommand(0.9);
		  		  break;
		  }
		  if(usart5_flag==1)
		  {
			  ActuatorStatus = 3;
		  }
		  else
		  {
			  ActuatorStatus = 1;
		  }
	  }
	  else if(ActuatorStatus==2)
	  {
		  switch(Step)
		  {
		  	  case 0:
		  		  CurrentCommand(0.0);
		  		  break;
		  	  case 1:
		  		  CurrentCommand(-1.1);
		  		  break;
		  	  case 2:
		  		  CurrentCommand(-1.0);
		  		  break;
		  	  case 3:
		  		  CurrentCommand(-0.9);
		  		  break;
		  }
		  if(usart5_flag==1)
		  {
			 ActuatorStatus = 0;
		  }
		  else
		  {
			  ActuatorStatus = 2;
		  }
	  }
	  else if(ActuatorStatus==3)
	  {
		  if(stroke1>=TRANS_POINT_A)//&&(stroke2>=TRANS_POINT_A))
		  {
			  if(direction==2)
			  {
					  ActuatorStatus = 2;
			  }
		  }
	  }
	  */
  }


  /*
  else if(ActuatorStatus==4)
  {
	  if((stroke1>=TRANS_POINT_0)||(stroke2>=TRANS_POINT_0))
	  {
		  //HAL_UART_Transmit(&huart5, (uint8_t *)"mcc=0,0\r\n", 9, 100);
		  CurrentCommand(CH1, 0.0);
		  CurrentCommand(CH2, 0.0);
		  ActuatorStatus = 3;
	  }
  }
  */
  /*
  if((ActuatorStatus==3)&&(direction==1))//&&((stroke1<TRANS_POINT_A)||(stroke2<TRANS_POINT_A))) // 2000
    {

      switch(Step)
      {
        case 0:
    	  //HAL_UART_Transmit(&huart5, (uint8_t *)"mcc=1.0,1.0\r\n", 13, 100);
          CurrentCommand(CH1, 1.0);
          CurrentCommand(CH2, 1.0);
    	  break;
        case 1:
    	  //HAL_UART_Transmit(&huart5, (uint8_t *)"mcc=1.1,1.1\r\n", 13, 100);
          CurrentCommand(CH1, 0.9);
          CurrentCommand(CH2, 0.9);
          break;
        case 2:
          //HAL_UART_Transmit(&huart5, (uint8_t *)"mcc=0.9,0.9\r\n", 13, 100);
          CurrentCommand(CH1, 1.0);
          CurrentCommand(CH2, 1.0);
          break;
        case 3:
          //HAL_UART_Transmit(&huart5, (uint8_t *)"mcc=0.7,0.7\r\n", 13, 100);
          CurrentCommand(CH1, 1.1);
          CurrentCommand(CH2, 1.1);
          break;
      }
      ActuatorStatus = 1;
    }
  else if((ActuatorStatus==1)&&((stroke1>=TRANS_POINT_A)||(stroke2>=TRANS_POINT_A)))  // 2000
    {
      //HAL_UART_Transmit(&huart5, (uint8_t *)"mcc=0,0\r\n", 9, 100);
	  CurrentCommand(CH1, 0.0);
	  CurrentCommand(CH2, 0.0);
      ActuatorStatus = 3;
    }

  if((ActuatorStatus==3)&&(direction==2))//&&(stroke1>TRANS_POINT_B)||(stroke2>TRANS_POINT_B))  // 400
    {

      switch(Step)
      {
        case 0:
        	//HAL_UART_Transmit(&huart5, (uint8_t *)"mcc=-1.0,-1.0\r\n", 15, 100);
        	CurrentCommand(CH1, -1.0);
        	CurrentCommand(CH2, -1.0);
        	break;
        case 1:
        	//HAL_UART_Transmit(&huart5, (uint8_t *)"mcc=-1.0,-1.0\r\n", 15, 100);
        	CurrentCommand(CH1, -0.9);
        	CurrentCommand(CH2, -0.9);
        	break;
        case 2:
        	//HAL_UART_Transmit(&huart5, (uint8_t *)"mcc=-0.8,-0.8\r\n", 15, 100);
        	CurrentCommand(CH1, -1.0);
        	CurrentCommand(CH2, -1.0);
        	break;
        case 3:
        	//HAL_UART_Transmit(&huart5, (uint8_t *)"mcc=-0.6,-0.6\r\n", 15, 100);
        	CurrentCommand(CH1, -1.1);
        	CurrentCommand(CH2, -1.1);
        	break;
      }
      ActuatorStatus = 2;
    }
  else if((ActuatorStatus==2)&&((stroke1<=TRANS_POINT_B)||(stroke2<=TRANS_POINT_B)))//||(stroke2<=TRANS_POINT_B)))  // 400
    {//HAL_UART_Transmit(&huart5, (uint8_t *)"mcc=0.1,0.1\r\n", 13, 100);
	  CurrentCommand(CH1, 0.0);
	  CurrentCommand(CH2, 0.0);
	  ActuatorStatus = 3;
    }
    */
  //printf("%d, %d, %d, %d\r\n", ActuatorStatus, stroke1, stroke2, force);
}

void Translate_Resist(void)
{
  static int ResistForce = 0;

  //HAL_UART_Transmit(&huart2, (uint8_t *)"0R", 2, 100);
  HAL_UART_Receive_DMA(&huart2, (uint8_t *)usart2_buffer, 10);

  //HAL_UART_Transmit(&huart5, (uint8_t *)"mp\r\n", 4, 100);
  //HAL_UART_Receive_DMA(&huart5, (uint8_t *)usart5_buffer, 13);
  if(abnormal)
  {
	  __HAL_UART_CLEAR_OREFLAG(&huart5);
	  HAL_UART_DMAStop(&huart5);
	  HAL_UART_Receive_DMA(&huart5, (uint8_t *)usart5_buffer, 13);
  }

  CheckPosition();

  switch(Step)
  {
    case 0:
    	ResistForce = RESIST_FORCE_DEFAULT;
        break;
    case 1:
    	ResistForce = RESIST_FORCE_STEP1;
        break;
    case 2:
    	ResistForce = RESIST_FORCE_STEP2;
        break;
    case 3:
    	ResistForce = RESIST_FORCE_STEP3;
        break;
    }

  if(force > ResistForce)
  {
	  direction = 2;  // backward
  }
  else if(force < (ResistForce*(-1)))
  {
      direction = 1;  // forward
  }
  else
  {
      direction = 0;  // stop
  }

  if(ActuatorStatus==0)
  {
      //HAL_UART_Transmit(&huart5, (uint8_t *)"mpc=500,500\r\n", 13, 100);
	  //PositionCommand(CH1, TRANS_POINT_0);
	  //PositionCommand(CH2, TRANS_POINT_0);
	  //ActuatorStatus = 4;
	  CurrentCommand(0.0);
	  ActuatorStatus = 3;
  }
  else if(ActuatorStatus==1)
  {
      switch(Step)
      {
      	case 0:
      		//HAL_UART_Transmit(&huart5, (uint8_t *)"mcc=1.0,1.0\r\n", 13, 100);
      		CurrentCommand(0.0);
      		break;
      	case 1:
      		//HAL_UART_Transmit(&huart5, (uint8_t *)"mcc=1.1,1.1\r\n", 13, 100);
      		CurrentCommand(1.1);
      		break;
        case 2:
        	//HAL_UART_Transmit(&huart5, (uint8_t *)"mcc=0.9,0.9\r\n", 13, 100);
        	CurrentCommand(1.0);
        	break;
        case 3:
        	//HAL_UART_Transmit(&huart5, (uint8_t *)"mcc=0.7,0.7\r\n", 13, 100);
        	CurrentCommand(0.9);
        	break;
        }
      ActuatorStatus = 3;
  }
  else if(ActuatorStatus==2)
  {
      switch(Step)
      {
        case 0:
        	//HAL_UART_Transmit(&huart5, (uint8_t *)"mcc=-1.0,-1.0\r\n", 15, 100);
        	CurrentCommand(0.0);
        	break;
        case 1:
        	//HAL_UART_Transmit(&huart5, (uint8_t *)"mcc=-1.0,-1.0\r\n", 15, 100);
        	CurrentCommand(-1.1);
        	break;
        case 2:
        	//HAL_UART_Transmit(&huart5, (uint8_t *)"mcc=-0.8,-0.8\r\n", 15, 100);
        	CurrentCommand(-1.0);
        	break;
        case 3:
        	//HAL_UART_Transmit(&huart5, (uint8_t *)"mcc=-0.6,-0.6\r\n", 15, 100);
        	CurrentCommand(-0.9);
        	break;
      }
      ActuatorStatus = 3;
  }
  else if(ActuatorStatus==3)
  {
	  if(direction==1)
	  {
		  ActuatorStatus = 1;
	  }
	  else if(direction==2)
	  {
		  ActuatorStatus = 2;
	  }
	  else
	  {
		  CurrentCommand(0.0);
		  ActuatorStatus = 3;
	  }
  }
/*
  else if(ActuatorStatus==4)
  {
	  if((stroke1>=TRANS_POINT_0)||(stroke2>=TRANS_POINT_0))
	  {
		  CurrentCommand(CH1, 0.0);
		  CurrentCommand(CH2, 0.0);
	      ActuatorStatus = 3;
	  }
      //HAL_UART_Transmit(&huart5, (uint8_t *)"mcc=0,0\r\n", 9, 100);
  }
  */
/*
  if((ActuatorStatus==3)&&(direction==1)&&((stroke1<TRANS_POINT_A)||(stroke2<TRANS_POINT_A))) // 2000
    {
      switch(Step)
      {
      	case 0:
      		//HAL_UART_Transmit(&huart5, (uint8_t *)"mcc=1.0,1.0\r\n", 13, 100);
      		CurrentCommand(CH1, 1.0);
      		CurrentCommand(CH2, 1.0);
      		break;
      	case 1:
      		//HAL_UART_Transmit(&huart5, (uint8_t *)"mcc=1.1,1.1\r\n", 13, 100);
      		CurrentCommand(CH1, 1.1);
      		CurrentCommand(CH2, 1.1);
      		break;
        case 2:
        	//HAL_UART_Transmit(&huart5, (uint8_t *)"mcc=0.9,0.9\r\n", 13, 100);
        	CurrentCommand(CH1, 1.0);
        	CurrentCommand(CH2, 1.0);
        	break;
        case 3:
        	//HAL_UART_Transmit(&huart5, (uint8_t *)"mcc=0.7,0.7\r\n", 13, 100);
        	CurrentCommand(CH1, 0.9);
        	CurrentCommand(CH2, 0.9);
        	break;
        }
      ActuatorStatus = 1;
    }
  else if((ActuatorStatus==1)&&((direction==0)||(stroke1>=TRANS_POINT_A)||(stroke2>=TRANS_POINT_A)))  // 2000
    {
	  //HAL_UART_Transmit(&huart5, (uint8_t *)"mcc=0,0\r\n", 9, 100);
	  CurrentCommand(CH1, 0.0);
	  CurrentCommand(CH2, 0.0);
      ActuatorStatus = 3;
    }

  if((ActuatorStatus==3)&&(direction==2)&&((stroke1>TRANS_POINT_B)||(stroke2>TRANS_POINT_B)))  // 400
    {
      switch(Step)
      {
        case 0:
        	//HAL_UART_Transmit(&huart5, (uint8_t *)"mcc=-1.0,-1.0\r\n", 15, 100);
        	CurrentCommand(CH1, -1.1);
        	CurrentCommand(CH2, -1.1);
        	break;
        case 1:
        	//HAL_UART_Transmit(&huart5, (uint8_t *)"mcc=-1.0,-1.0\r\n", 15, 100);
        	CurrentCommand(CH1, -1.1);
        	CurrentCommand(CH2, -1.1);
        	break;
        case 2:
        	//HAL_UART_Transmit(&huart5, (uint8_t *)"mcc=-0.8,-0.8\r\n", 15, 100);
        	CurrentCommand(CH1, -1.0);
        	CurrentCommand(CH2, -1.0);
        	break;
        case 3:
        	//HAL_UART_Transmit(&huart5, (uint8_t *)"mcc=-0.6,-0.6\r\n", 15, 100);
        	CurrentCommand(CH1, -0.9);
        	CurrentCommand(CH2, -0.9);
        	break;
      }
      ActuatorStatus = 2;
    }
  else if((ActuatorStatus==2)&&((direction==0)||(stroke1<=TRANS_POINT_B)||(stroke2<=TRANS_POINT_B)))  // 500
    {
	  //HAL_UART_Transmit(&huart5, (uint8_t *)"mcc=0.1,0.1\r\n", 13, 100);
	  CurrentCommand(CH1, 0.0);
	  CurrentCommand(CH2, 0.0);
      ActuatorStatus = 3;
    }
*/
  //printf("%d, %d, %d, %d\r\n", ActuatorStatus, stroke1, stroke2, force);
}

void PushOff(void)
{
  static uint8_t torque = 0;
    switch(Step)
    {
      case 0:
	torque = RESIST_TORQUE_DEFAULT;
        break;
      case 1:
	torque = RESIST_TORQUE_STEP1;
        break;
      case 2:
        torque = RESIST_TORQUE_STEP2;
        break;
      case 3:
        torque = RESIST_TORQUE_STEP3;
        if(!(SysStatus&MASK_MBRK))
                {
                	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET);
                    SysStatus|=MASK_MBRK;
                }
        break;
    }
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, torque);

    EncoderCounter = TIM2->CNT;
}

void wheel(void)
{
  static uint32_t OldPos = 0;
  static uint8_t torque = 0;

  switch(Step)
  {
    case 0:
    	torque = RESIST_TORQUE_DEFAULT;
        break;
    case 1:
        torque = RESIST_TORQUE_STEP1;
        break;
    case 2:
        torque = RESIST_TORQUE_STEP2;
        break;
    case 3:
        torque = RESIST_TORQUE_STEP3;
        if(!(SysStatus&MASK_MBRK))
        {
        	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET);
            SysStatus|=MASK_MBRK;
        }
        break;
    }
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, torque);

  EncoderCounter = TIM2->CNT;

  if(!(SysStatus&MASK_CL))
    {
	  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_RESET);
      SysStatus|=MASK_CL;
    }
  HAL_Delay(2);
  //printf("%d %d %d\r\n", EncoderCounter, OldPos, (EncoderCounter-OldPos));
  OldPos = EncoderCounter;
}

void Adjust_Height_Up(void)
{
  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);
  HAL_Delay(1);
  HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_3);
  HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_4);
  HAL_Delay(1);
}

void Adjust_Height_Down(void)
{
	//CurrentCommand(CH1, -1.1);
	//CurrentCommand(CH2, -1.1);
  //HAL_UART_Transmit(&huart5, (uint8_t *)"cc=-0.5\r\n", 9, 100);

  //HAL_UART_Transmit(&huart5, (uint8_t *)"mp\r\n", 4, 100);
  //HAL_UART_Receive_DMA(&huart5, (uint8_t *)usart5_buffer, 13);
  //HAL_Delay(2);

  //HAL_UART_Transmit(&huart5, (uint8_t *)"mcc=-1.0,-1.0\r\n", 15, 100);

  //printf("%d\r\n", stroke);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
