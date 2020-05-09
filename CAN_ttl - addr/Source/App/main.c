/*************************************
		杭州光子物联网技术有限公司
		https://gzwelink.taobao.com
				2015.08.01
**************************************/
#include "stm32f10x.h"
#include "ALL_Includes.h"
#include "uart.h"
#include "stdio.h"
#include "BitBand.h"
#include "JY901.h"
#include "led.h"
#include "software_iic.h"

extern unsigned char uart_rec_buffer[ ];
extern unsigned char index_uart ;
extern unsigned char uart_buffer_head_ok;
extern unsigned char cmd_in;

unsigned char  x,y,z ;
unsigned char databuff[3];

unsigned char my_addr=0;
uint8_t n;
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;
vu32 ret; /* for return of the interrupt handling */
volatile TestStatus TestRx;
ErrorStatus HSEStartUpStatus;

TestStatus CAN_Polling(void);
TestStatus CAN_Interrupt(void);

void GPIO_Configuration(void)
{
	  GPIO_InitTypeDef GPIO_InitStructure;
  	/* Configure CAN pin: RX */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  	/* Configure CAN pin: TX */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
	  /* Configure led 1 2  */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* Configure led 1 2  */
  	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	 /* Configure led 1 2  */
  	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3 ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
		
		/* Configure led 1 2  */
  	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2 ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
}

//系统中断管理
void NVIC_Configuration(void)
{ 
	NVIC_InitTypeDef NVIC_InitStructure;

  	/* Configure the NVIC Preemption Priority Bits */  
  	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

	#ifdef  VECT_TAB_RAM  
	  /* Set the Vector Table base location at 0x20000000 */ 
	  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
	#else  /* VECT_TAB_FLASH  */
	  /* Set the Vector Table base location at 0x08000000 */ 
	  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
	#endif

	/* enabling interrupt */
  	NVIC_InitStructure.NVIC_IRQChannel=USB_LP_CAN1_RX0_IRQn;;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
}

//配置系统时钟,使能各外设时钟
void RCC_Configuration(void)
{
	SystemInit();	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA 
                           |RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC
                           |RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE
						   |RCC_APB2Periph_ADC1  | RCC_APB2Periph_AFIO 
                           |RCC_APB2Periph_SPI1, ENABLE );
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 
                           |RCC_APB1Periph_USART3|RCC_APB1Periph_TIM2	                           
                           , ENABLE );
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	/* CAN Periph clock enable */
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
}

  char message[100];
//void uart1_send_str(  char *str)
//{
//unsigned char i=0;
//	while(str[i]!='\0')
//	{
//   UART1_WriteByte(str[i++]);
//	}
//}
 unsigned char hex3[2],hex2[2],hex1[2],dat[3];
//??? str[0]
void HexToStr(unsigned char val,unsigned char *str)
{
	//str[0]=_crol_(val,4) & 0xf;
	str[0]=(val& 0xf0)>>4;
	str[1]=val & 0xf;
	if (str[0]>9) str[0] +=55; else str[0] +=48;
	if (str[1]>9) str[1] +=55; else str[1] +=48;	
	//str[2]=0;
}//*/

//high in str[0];
unsigned char StrToHex(unsigned char val,unsigned char *str)
{
	unsigned int i=0;
	//str[0]=_crol_(val,4) & 0xf;
	if(str[0]>='0'&&str[0]<='9') str[0]=str[0]-'0';
	if(str[1]>='0'&&str[1]<='9') str[1]=str[1]-'0';
	if(str[0]>='a'&&str[0]<='f') str[0]=str[0]-'a'+10;
	if(str[1]>='a'&&str[1]<='f') str[1]=str[1]-'a'+10;
	if(str[0]>='A'&&str[0]<='F') str[0]=str[0]-'A'+10;
	if(str[1]>='A'&&str[1]<='F') str[1]=str[1]-'A'+10;
	i=str[0];
	i=(i<<4)+str[1];
	return (i);
}//*/

u8 CanTxData(void)
{
  CanTxMsg TxMessage;
  CanRxMsg RxMessage;
  u32 i = 0;
  u8 TransmitMailbox;
  /* transmit */
  TxMessage.StdId=0xf4;
  TxMessage.RTR=0x04;
  TxMessage.IDE=0x04;
	TxMessage.ExtId=0x1765432f;
	 
  TxMessage.DLC=8;
  TxMessage.Data[0]=9;
  TxMessage.Data[1]=2;
  TxMessage.Data[2]=3;
  TxMessage.Data[3]=4;
  TxMessage.Data[4]=5;
  TxMessage.Data[5]=6;
  TxMessage.Data[6]=7;
  TxMessage.Data[7]=1;

  TransmitMailbox=CAN_Transmit(CAN1,&TxMessage);
  i = 0;
  while((CAN_TransmitStatus(CAN1,TransmitMailbox) != CANTXOK) && (i != 0xFF))
  {
    i++;
  }

  i = 0;
  while((CAN_MessagePending(CAN1,CAN_FIFO0) < 1) && (i != 0xFF))
  {
    i++;
  }

}

u8 Can_send_Data(unsigned int ext_frame_id,unsigned char *data,unsigned char data_n)
{
  CanTxMsg TxMessage;
  CanRxMsg RxMessage;
  u32 i = 0;
  u8 TransmitMailbox;
  /* transmit */
  TxMessage.StdId=0xf4;
  TxMessage.RTR=0x04;
  TxMessage.IDE=0x04;
	TxMessage.ExtId=ext_frame_id;
	 
  TxMessage.DLC=data_n;
	for(  i=0;i<data_n;i++)
     TxMessage.Data[i]=data[i];
  

  TransmitMailbox=CAN_Transmit(CAN1,&TxMessage);
  i = 0;
  while((CAN_TransmitStatus(CAN1,TransmitMailbox) != CANTXOK) && (i != 0xFF))
  {
    i++;
  }

  i = 0;
  while((CAN_MessagePending(CAN1,CAN_FIFO0) < 1) && (i != 0xFF))
  {
    i++;
  }
 ///////////////  CAN ?? ??
			 if(CAN_GetFlagStatus(CAN1, CAN_FLAG_BOF)==SET)
        {
                                // CAN_Config();
                        //        CAN_ClearFlag(CANx, CAN_FLAG_BOF);
           CAN1->MCR|=1;
           CAN1->MCR&=0xfffffffe;

           sprintf(message,"get can1 off!resend!\r\n"    );
			      uart1_send_str(message);
					 TransmitMailbox=CAN_Transmit(CAN1,&TxMessage);
           i = 0;
           while((CAN_TransmitStatus(CAN1,TransmitMailbox) != CANTXOK) && (i != 0xFF))
           {
            i++;
           }

            i = 0;
           while((CAN_MessagePending(CAN1,CAN_FIFO0) < 1) && (i != 0xFF))
           {
            i++;
           }
					
					
          } // end of can err judge
}

unsigned char cmp_data(  char *data1,unsigned char *data2,unsigned char n)
{
unsigned char i=0;
	for(i=0;i<n;i++)
	{
	 if(data1[i]!=data2[i]) return i;
	}
	return i;
}

//配置所有外设
void Init_All_Periph(void)
{
	Delay_Init(72);
	RCC_Configuration();
	//LcdGpioConfig();
	//Init_lcd();	
	GPIO_Configuration();
	NVIC_Configuration();
}
CanRxMsg RxMessage;
 

                                 //low high 
unsigned char     my_bcl_data[]= {0x34,0x08,0x5a,0x0f,0x01,0x00,0x00,0x00};
//  0834  210v

void   can_dat_uart1()
{
int i=0;
 if(RxMessage.DLC)//接收长度有效
		 {
			  
			 
			 
			 if(RxMessage.ExtId==0x11390000)
			 {
			   TestRx=Can_send_Data(0x12800000+my_addr,my_bcl_data,8);   
				 RxMessage.DLC=0;
				 delay_us(50);
			 
			 }
			 sprintf(message,"@%d,CAN,n=%d:,ID=%8x,data%2x%2x%2x%2x%2x%2x%2x%2x#\r\n" ,my_addr,RxMessage.DLC,RxMessage.ExtId,RxMessage.Data [0],RxMessage.Data [1],RxMessage.Data [2],RxMessage.Data [3],RxMessage.Data [4],RxMessage.Data [5],RxMessage.Data [6],RxMessage.Data [7]  );
			 for(n=0;n<50;n++)                    ///  空格换 0
			 
			 {
				 if(*(message+n)==' ')*(message+n)='0';
				 if(*(message+n)=='#')break;
			 }
			 
			
			 
			 uart1_send_str(message);
			 
			 ///////////////////////////////////
			 RxMessage.DLC=0;
		 }
		 // release bus
		 delay_us(200);
		 
}


unsigned int get_can_id_from_uart(void)
{
	int i=0;
 hex1[0]=uart_rec_buffer[14];
 hex1[1]=uart_rec_buffer[15];	
 i=StrToHex(0,hex1);
 hex1[0]=uart_rec_buffer[16];
 hex1[1]=uart_rec_buffer[17];	
 i=(i<<8)+StrToHex(0,hex1);
 hex1[0]=uart_rec_buffer[18];
 hex1[1]=uart_rec_buffer[19];	
 i=(i<<8)+StrToHex(0,hex1);
 hex1[0]=uart_rec_buffer[20];
 hex1[1]=uart_rec_buffer[21];	
 i=(i<<8)+StrToHex(0,hex1);	
	
	return(i);
}
unsigned char  get_can_data(unsigned char *str)
{
 unsigned char n=0,i=0;
	n=uart_rec_buffer[9+2]-'0';
	if(n>=8) n=8;
	for(i=0;i<n;i++)
	{
	 hex1[0]=uart_rec_buffer[23+2*i];
   hex1[1]=uart_rec_buffer[23+2*i+1];	
   str[i]=StrToHex(0,hex1);
	}
	
return(n);
}

#define myaddr 0


/*******************************************************************************
* Function Name  : CAN_Polling
* Description    : Configures the CAN and transmit and receive by polling
* Input          : None
* Output         : None
* Return         : PASSED if the reception is well done, FAILED in other case
*******************************************************************************/
TestStatus CAN_Polling(void)
{
  CAN_InitTypeDef        CAN_InitStructure;
  CAN_FilterInitTypeDef  CAN_FilterInitStructure;
  CanRxMsg RxMessage;
  u8 TransmitMailbox;

  /* CAN register init */
  CAN_DeInit(CAN1);
  CAN_StructInit(&CAN_InitStructure);

  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM=DISABLE;
  CAN_InitStructure.CAN_ABOM=DISABLE;
  CAN_InitStructure.CAN_AWUM=DISABLE;
  CAN_InitStructure.CAN_NART=DISABLE;
  CAN_InitStructure.CAN_RFLM=DISABLE;
  CAN_InitStructure.CAN_TXFP=DISABLE;
  CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
  CAN_InitStructure.CAN_BS1=CAN_BS1_8tq;
  CAN_InitStructure.CAN_BS2=CAN_BS2_7tq;
  CAN_InitStructure.CAN_Prescaler=9;
  CAN_Init(CAN1,&CAN_InitStructure);

  /* CAN filter init */
  CAN_FilterInitStructure.CAN_FilterNumber=0;
  CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;
  CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);



  /* receive */
  RxMessage.StdId=0x00;
  RxMessage.IDE=CAN_ID_STD;
  RxMessage.DLC=0;
  RxMessage.Data[0]=0x00;
  RxMessage.Data[1]=0x00;
  CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);

  if (RxMessage.StdId!=0x11) return FAILED;

  if (RxMessage.IDE!=CAN_ID_STD) return FAILED;

  if (RxMessage.DLC!=2) return FAILED;

  if ((RxMessage.Data[0]<<8|RxMessage.Data[1])!=0xCAFE) return FAILED;
  
  return PASSED; /* Test Passed */
}

/*******************************************************************************
* Function Name  : CAN_Interrupt
* Description    : Configures the CAN and transmit and receive by interruption
* Input          : None
* Output         : None
* Return         : PASSED if the reception is well done, FAILED in other case
*******************************************************************************/
TestStatus CAN_Interrupt(void)
{
  CAN_InitTypeDef        CAN_InitStructure;
  CAN_FilterInitTypeDef  CAN_FilterInitStructure;
  CanTxMsg TxMessage;
  u32 i = 0;

  /* CAN register init */
  CAN_DeInit(CAN1);
  CAN_StructInit(&CAN_InitStructure);

  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM=DISABLE;
  CAN_InitStructure.CAN_ABOM=DISABLE;
  CAN_InitStructure.CAN_AWUM=DISABLE;
  CAN_InitStructure.CAN_NART=DISABLE;
  CAN_InitStructure.CAN_RFLM=DISABLE;
  CAN_InitStructure.CAN_TXFP=DISABLE;
  CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
  CAN_InitStructure.CAN_BS1=CAN_BS1_8tq;
  CAN_InitStructure.CAN_BS2=CAN_BS2_7tq;
  CAN_InitStructure.CAN_Prescaler=9;
  CAN_Init(CAN1,&CAN_InitStructure);

  /* CAN filter init */
  CAN_FilterInitStructure.CAN_FilterNumber=1;
  CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;
  CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);

  /* CAN FIFO0 message pending interrupt enable */ 
  CAN_ITConfig(CAN1,CAN_IT_FMP0, ENABLE);


  return (TestStatus)0;
}

/*******************************************************************************
* Function Name  : USB_LP_CAN_RX0_IRQHandler
* Description    : This function handles USB Low Priority or CAN RX0 interrupts 
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB_LP_CAN1_RX0_IRQHandler(void)
{

  CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);

}

extern unsigned char index_uart3;
extern unsigned char uart_rec_buffer3[];
unsigned char cmd_485_10read[]={8,0x10,0x04,0x00,0x10,0x00,0x02,0x73,0x4f};//read id=10
unsigned char cmd_485_11read[]={8,0x10,0x04,0x00,0x10,0x00,0x02,0x73,0x4f};//read id=10
unsigned char cmd_485_12read[]={8,0x10,0x04,0x00,0x10,0x00,0x02,0x73,0x4f};//read id=10
unsigned char cmd_485_13read[]={8,0x10,0x04,0x00,0x10,0x00,0x02,0x73,0x4f};//read id=10
unsigned char cmd_485_id_change[]={8,0x10,0x06,0x00,0x00,0x00,0x11,0x73,0x4f};//read id=10


int encoder_quanshu[3];
int encoder_angle[3];
extern struct SQ       stcQ;
extern struct SAcc 		stcAcc;
extern struct SAngle 	stcAngle;
extern struct SGyro 		stcGyro;

float eular_x,eular_y,eular_z;


extern unsigned char uart_rec_buffer2[ ];
extern unsigned char index_uart2 ;
extern unsigned char uart_rec_buffer[ ];
void process_cmd1()
{
	unsigned char cmp_len,i,check,cmd_buf[30],temp_c1;
	unsigned	int plc_addr,plc_val,temp_int1,crc_check;
	
	
	
	////////// rs485cmd send  modbus /////////////////
	sprintf(message,"@%d,read_sensor_data,#" , my_addr );   
	  cmp_len=strlen(message)-1;
		if( cmp_data(message,uart_rec_buffer,cmp_len)==cmp_len)		
			{
					eular_x=stcAngle.Angle[0];
      eular_y=stcAngle.Angle[1];
      eular_z=stcAngle.Angle[2];
			  	 eular_x=(eular_x/32768.0)*180;
					 eular_y=(eular_y/32768.0)*180;
					 eular_z=(eular_z/32768.0)*180;
			 sprintf(message,"@%d,c1,%d,%d,c2,%d,%d,c3,%d,%d,angle_xyz,%3f,%3f,%3f,#\r\n"
				,my_addr,encoder_quanshu[0],encoder_angle[0],encoder_quanshu[1],encoder_angle[1],encoder_quanshu[2],encoder_angle[2],eular_x,eular_y,eular_z  ); 
				uart1_send_str(message); 
			}
	
	
	sprintf(message,"@%d,testencoder,#" , my_addr );   
	  cmp_len=strlen(message)-1;
		if( cmp_data(message,uart_rec_buffer,cmp_len)==cmp_len)		
			{
				 cmp_len=cmp_len;
				temp_c1=uart_rec_buffer[cmp_len]-'0';
				for(i=1;i<10;i++)
				{
					if(uart_rec_buffer[cmp_len+i]==',') break;
				  temp_c1=temp_c1*10+uart_rec_buffer[cmp_len+i]-'0';					
				}
				cmd_485_10read[1]=temp_c1;
				crc_check=CrcCal(cmd_485_10read,1,6);
				cmd_485_10read[7]=(crc_check>>8)&0x00ff;
				cmd_485_10read[8]=crc_check&0x00ff;
			  // UART1_WriteBytes(cmd_485_10,1,cmd_485_10[0]);
				en_485_send;
				delay_ms(1);
				index_uart2=0;
				 UART2_WriteBytes(cmd_485_10read,1,cmd_485_10read[0]);
				delay_ms(1);
				en_485_rec;
				delay_ms(10);
				 // UART1_WriteBytes(uart_rec_buffer2,0,index_uart2);
				crc_check=CrcCal(uart_rec_buffer2,0,index_uart2-3);
					sprintf(message,"%d crc=%2x,%2x,index uart2=%d,input crc=%2x,%2x, \r\n",temp_c1,(crc_check>>8)&0x00ff,crc_check&0x00ff,index_uart2,uart_rec_buffer2[index_uart2-2],uart_rec_buffer2[index_uart2-1]); 
				uart1_send_str(message); 
			}
			////////////
			sprintf(message,"@%d,change_id,#" , my_addr );   
	    cmp_len=strlen(message)-1;
		  if( cmp_data(message,uart_rec_buffer,cmp_len)==cmp_len)		
			 {
				  
				 cmp_len=cmp_len;
				 temp_c1=uart_rec_buffer[cmp_len]-'0';
				 for(i=1;i<10;i++)
				 {
					if(uart_rec_buffer[cmp_len+i]==',') break;
				  temp_c1=temp_c1*10+uart_rec_buffer[cmp_len+i]-'0';					
				 }
				 cmd_485_id_change[1]=temp_c1;
				 
				 cmp_len=cmp_len+i+1;
				 temp_c1=uart_rec_buffer[cmp_len]-'0';
				 for(i=1;i<10;i++)
				 {
					if(uart_rec_buffer[cmp_len+i]==',') break;
				  temp_c1=temp_c1*10+uart_rec_buffer[cmp_len+i]-'0';					
				 }
				 cmd_485_id_change[6]=temp_c1;
				 crc_check=CrcCal(cmd_485_id_change,1,6);
				 cmd_485_id_change[7]=(crc_check>>8)&0x00ff;
				 cmd_485_id_change[8]=crc_check&0x00ff;
				 
			  // UART1_WriteBytes(cmd_485_10,1,cmd_485_10[0]);
				en_485_send;
				delay_ms(1);
				index_uart2=0;
				 UART2_WriteBytes(cmd_485_id_change,1,cmd_485_id_change[0]);
				delay_ms(1);
				en_485_rec;
				delay_ms(10);
				 // UART1_WriteBytes(uart_rec_buffer2,0,index_uart2);
				crc_check=CrcCal(uart_rec_buffer2,index_uart2-2);
					sprintf(message,"%d to %d,crc=%2x,%2x,%d,\r\n", cmd_485_id_change[1],cmd_485_id_change[6],(crc_check>>8)&0x00ff,crc_check&0x00ff,index_uart2);
				 
				uart1_send_str(message);
				for(i=0;i<index_uart2;i++)
				{
				sprintf(message,"%2x  ",uart_rec_buffer2[i] );
				 
				uart1_send_str(message);
				}
				
			}
			
			////////////////////////////////////////////////
	
	
	 
	//////////end of rs232 cmd send/////////////////
			
}



unsigned int timer1=0;


int auto_upload=1;

int main(void)
{  
	unsigned char i,data[10],can_data_n_send,can_tx_data[10],k;
	unsigned int data_id,crc_check;
	unsigned char bms_step=0;
	float max_chager_voletage=0;
	float min_chager_voletage=0;
	float max_chager_current=0;
	unsigned char ccs_cnt=0;
	int para_set=0;
	unsigned long main_cnt=0;
	unsigned char encoder_scan_index=0;
	Init_All_Periph();
	LED_GPIO_Config();
	
	en_485_rec;
	
	UART1_Config(115200);
	UART2_Config(115200);
  USART3_Init(9600);
 
 auto_upload=1;
sprintf(message,"@%d,reset\r\n" ,my_addr  );
			 uart1_send_str(message);
	
	
  TestRx = CAN_Interrupt();
 
	delay_ms(2000);
 
		
 sprintf(message,"@%d,CAN,n=%d:,ID=%8x \r\n" ,my_addr,RxMessage.DLC,RxMessage.ExtId   );
			 uart1_send_str(message);
	delay_ms(2000);
	 
	
	I2C_Initializes();
	
	EEPROM_WriteByte(0,0);
	
	
	sprintf(message,"@%d,iic w ok!\r\n" ,my_addr  );
			 uart1_send_str(message);
 	while(1)
  	{
			main_cnt++;
		  //can_dat_uart1(); // time
			delay_us(1);
			 
       
			 
			 if(cmd_in==1)
			 {
				 process_cmd1();
				   led2=!led2;
					cmd_in=0;
			 }// end of cmd in
			 
			 
			  
  	}// end of main while
		
	
}


