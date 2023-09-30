/**
  ******************************************************************************
  * @file    main.c
  * @author  MCU Application Team
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) Puya Semiconductor Co.
  * All rights reserved.</center></h2>
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "py32f030xx_ll_Start_Kit.h"

//#include "tjc_usart_hmi.h"

/* Private define head--------------------------------------------------------*/
#if 1
//---cmd

//---uart

//gpo
//---system hold
#define	SYS_HOLD_PORT		GPIOB
#define	SYS_HOLD_PIN		LL_GPIO_PIN_7

//---LED
#define	LED_PORT		GPIOF
#define	LED_PIN			LL_GPIO_PIN_3

//---motor1,motor2
#define	MOTO1_PORT		GPIOA
#define	MOTO1_A_PIN		LL_GPIO_PIN_9
#define	MOTO1_B_PIN		LL_GPIO_PIN_10

#define	MOTO2_PORT		GPIOA
#define	MOTO2_A_PIN		LL_GPIO_PIN_11
#define	MOTO2_B_PIN		LL_GPIO_PIN_12

#define	MOTO3_PORT		GPIOB
#define	MOTO3_A_PIN		LL_GPIO_PIN_3
#define	MOTO3_B_PIN		LL_GPIO_PIN_4

#define	MOTO4_PORT		GPIOB
#define	MOTO4_A_PIN		LL_GPIO_PIN_5
#define	MOTO4_B_PIN		LL_GPIO_PIN_6

#define	MOTO_VIBRATE_PORT		GPIOA
#define	MOTO_VIBRATE_PIN			LL_GPIO_PIN_8


//---sensor clk
#define SENX_PORT				GPIOA
#define SEN1_CLK_PIN		LL_GPIO_PIN_2
#define SEN2_CLK_PIN		LL_GPIO_PIN_4
#define SEN3_CLK_PIN		LL_GPIO_PIN_6

//gpi start
#if 1
//---sensor data
#define	S1_DOUT_PIN	  	LL_GPIO_PIN_1
#define	S2_DOUT_PIN	  	LL_GPIO_PIN_3
#define	S3_DOUT_PIN	  	LL_GPIO_PIN_5

//---key_H
#define KEY_HIN_PORT		GPIOA
#define KEY_HIN_PIN			LL_GPIO_PIN_7
#define KEY_HOT_PORT		GPIOB
#define KEY_HOT_PIN			LL_GPIO_PIN_0

//---key_V
#define KEY_VX_PORT			GPIOB
#define KEY_VUP_PIN			LL_GPIO_PIN_1
#define KEY_VDN_PIN			LL_GPIO_PIN_2
#endif
//gpi end









//runMode
#define MANUAL 			0
#define SETTING   	1
#define AUTO   			2

//key value
#define KEY_NONE		  		0
#define KEY_X_IN  	  		1	//horizontal in
#define KEY_X_OUT					2	//horizontal out
#define KEY_Y_UP  	  		3	//vertical in
#define KEY_Y_DOWN				4	//vertical out
#define KEY_Z_IN  	  		5	//tight
#define KEY_Z_OUT					6	//loosen
#define KEY_U_UP  	  		7	//rotate up
#define KEY_U_DOWN				8	//rotate down

//motor active
#define MOTOR_NONE		  	0
#define MOTOR_X_IN  	  	1	//horizontal in
#define MOTOR_X_OUT				2	//horizontal out
#define KMOTOR_Y_UP  	  	3	//vertical in
#define MOTOR_Y_DOWN			4	//vertical out
#define MOTOR_Z_IN  	  	5	//tight
#define MOTOR_Z_OUT				6	//loosen
#define MOTOR_U_UP  	  	7	//rotate up
#define MOTOR_U_DOWN			8	//loosen
#define MOTOR_ALARM				9	//rotate down
#endif
/* Private define end---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
#if 1
uint8_t	runMode=MANUAL;	//MANUAL ; AUTO	;SETTING
uint8_t	flagReadSetting;	//read setting value only 1 times

uint8_t flagSys=1;		//1=Someone has sat down
uint8_t flagPwrOff=0;	//1=start timing 5s for power off
uint8_t flagAlarm=0;	//1=start timing for alarm

uint8_t flagM1=0;	//0=off;1=motor1 inner;2=motor1 outter;
__IO uint8_t flagMH=0;	//0=off;1=horizontal inner;2=horizontal outter;
__IO uint8_t flagMV=0;	//0=off;1=vertical up;2=vertical down;
__IO uint8_t flagML=0;	//0=off;1=lean up;2=lean down;
__IO uint8_t flagM5=0;	//0=off;1=vibration;




uint8_t		valKey=KEY_NONE;		//NONE;KEY_(X_IN;X_OUT;Y_UP;Y_DOWN;Z_IN;Z_OUT)

uint8_t		setValAlarm=0;			//x Mins
uint8_t		valAlarm=0;					//

uint8_t		flagMotor=0;		//0=no alarm; 1=alarm

uint16_t	senxRefresh=0;	//0.5s refresh 1 times

uint32_t	valSenS=0;	//S=seat sensor
uint32_t	valSenH=0;	//H=horizontal sensor
uint32_t	valSenV=0;	//V=vertical	sensor

uint32_t	valSenHsaved=0;	//setting value of horizontal sensor
uint32_t	valSenVsaved=0;	//setting value of V=vertical	sensor

//uart
#if 1
uint8_t aTxBuffer[] = "x0.val=000000";	//12B
uint8_t aTxBufDebug[] = "mHinS000";	//8B				
uint8_t aTxBufDebug2[] = "flgmH=0000";	//10B		


uint8_t aRxBuffer[10];

uint8_t *TxBuff = NULL;
__IO uint16_t TxSize = 0;
__IO uint16_t TxCount = 0;

uint8_t *RxBuff = NULL;
__IO uint16_t RxSize = 0;
__IO uint16_t RxCount = 0;

__IO ITStatus UartReady = RESET;



#endif


#endif
/* Private variables ---------------------------------------------------------*/

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function head------------------------------------------------------*/
#if 1
//clk init
void sysInit(void)
{
//开SYSCFG和PWR时钟
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
	
//配置系统时钟 */

  LL_RCC_HSI_Enable();	  /* 使能HSI */
  while(LL_RCC_HSI_IsReady() != 1){ }

  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);  /* 设置 AHB 分频*/

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);  /* 配置HSISYS作为系统时钟源 */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS) { }

  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);  /* 设置 APB1 分频*/
  LL_Init1msTick(8000000);

  LL_SetSystemCoreClock(8000000);	  /* 更新系统时钟全局变量SystemCoreClock(也可以通过调用SystemCoreClockUpdate函数更新) */
}


//sys hold
void sysHold(void)
{
	LL_GPIO_SetOutputPin(SYS_HOLD_PORT, SYS_HOLD_PIN);	
}
//sys off
void sysOff(void)
{
	LL_GPIO_ResetOutputPin(SYS_HOLD_PORT, SYS_HOLD_PIN);	
}
//led off
void ledOff(void)
{
	LL_GPIO_SetOutputPin(LED_PORT, LED_PIN);	//LED OFF
}
//led on
void ledOn(void)
{
	LL_GPIO_ResetOutputPin(LED_PORT, LED_PIN);	//LED ON
}
//led toggle
void ledToggle(void)
{
	LL_GPIO_TogglePin(LED_PORT, LED_PIN);	//LED toggle
}
void tToggle(void)
{
	LL_GPIO_TogglePin(MOTO2_PORT, MOTO2_A_PIN);	//LED toggle
}

void MOTO_VIBRATE_ON(void)
{
	LL_GPIO_SetOutputPin(MOTO_VIBRATE_PORT, MOTO_VIBRATE_PIN);
}
void MOTO_VIBRATE_OFF(void)
{
	LL_GPIO_ResetOutputPin(MOTO_VIBRATE_PORT, MOTO_VIBRATE_PIN);
}

		
//=============================================================================
void M1_FW(void)
{
	LL_GPIO_SetOutputPin(MOTO1_PORT, MOTO1_A_PIN);
	LL_GPIO_ResetOutputPin(MOTO1_PORT, MOTO1_B_PIN);
}
void M1_FF(void)
{
	LL_GPIO_ResetOutputPin(MOTO1_PORT, MOTO1_A_PIN);
	LL_GPIO_SetOutputPin(MOTO1_PORT, MOTO1_B_PIN);
}
void M1_OFF(void)
{
	LL_GPIO_ResetOutputPin(MOTO1_PORT, MOTO1_A_PIN);
	LL_GPIO_ResetOutputPin(MOTO1_PORT, MOTO1_B_PIN);
}
//=============================================================================
void mHin(void)
{
	LL_GPIO_SetOutputPin(MOTO2_PORT, MOTO2_A_PIN);
	LL_GPIO_ResetOutputPin(MOTO2_PORT, MOTO2_B_PIN);
}
void mHout(void)
{
	LL_GPIO_ResetOutputPin(MOTO2_PORT, MOTO2_A_PIN);
	LL_GPIO_SetOutputPin(MOTO2_PORT, MOTO2_B_PIN);
}
void mHoff(void)
{
	LL_GPIO_ResetOutputPin(MOTO2_PORT, MOTO2_A_PIN);
	LL_GPIO_ResetOutputPin(MOTO2_PORT, MOTO2_B_PIN);
}
//=============================================================================	
void mVup(void)
{
	LL_GPIO_SetOutputPin(MOTO3_PORT, MOTO3_A_PIN);
	LL_GPIO_ResetOutputPin(MOTO3_PORT, MOTO3_B_PIN);
}
void mVdown(void)
{
	LL_GPIO_ResetOutputPin(MOTO3_PORT, MOTO3_A_PIN);
	LL_GPIO_SetOutputPin(MOTO3_PORT, MOTO3_B_PIN);
}
void mVoff(void)
{
	LL_GPIO_ResetOutputPin(MOTO3_PORT, MOTO3_A_PIN);
	LL_GPIO_ResetOutputPin(MOTO3_PORT, MOTO3_B_PIN);
}

void mLup(void)	//lean
{
	LL_GPIO_SetOutputPin(MOTO4_PORT, MOTO4_A_PIN);
	LL_GPIO_ResetOutputPin(MOTO4_PORT, MOTO4_B_PIN);
}
void mLdown(void)
{
	LL_GPIO_ResetOutputPin(MOTO4_PORT, MOTO4_A_PIN);
	LL_GPIO_SetOutputPin(MOTO4_PORT, MOTO4_B_PIN);
}
void mLoff(void)
{
	LL_GPIO_ResetOutputPin(MOTO4_PORT, MOTO4_A_PIN);
	LL_GPIO_ResetOutputPin(MOTO4_PORT, MOTO4_B_PIN);
}
//sensor
uint32_t readSenS(void)		//get value of senS(S=seat)
{
	uint32_t sensValue=0;
#if 0	
	uint8_t i;
	
	LL_GPIO_ResetOutputPin(SENX_PORT, SEN1_CLK_PIN);
	while( LL_GPIO_IsInputPinSet(SENX_PORT, S1_DOUT_PIN));
	for(i=0;i<24;i++)
		{
			LL_GPIO_SetOutputPin(SENX_PORT, SEN1_CLK_PIN);
			sensValue=sensValue<<1;
			LL_GPIO_ResetOutputPin(SENX_PORT, SEN1_CLK_PIN);
			if(LL_GPIO_IsInputPinSet(SENX_PORT, S1_DOUT_PIN))	
				{
					sensValue++;
				}
		}
	LL_GPIO_SetOutputPin(SENX_PORT, SEN1_CLK_PIN);
	sensValue=sensValue^0x800000;
	LL_GPIO_ResetOutputPin(SENX_PORT, SEN1_CLK_PIN);	
#endif	
	return(sensValue);	
}	

uint32_t readSenH(void)
{
	uint32_t sensValue=0;
#if 0	
	uint8_t i;
	
	LL_GPIO_ResetOutputPin(SENX_PORT, SEN2_CLK_PIN);
	while( LL_GPIO_IsInputPinSet(SENX_PORT, S2_DOUT_PIN));
	for(i=0;i<24;i++)
		{
			LL_GPIO_SetOutputPin(SENX_PORT, SEN2_CLK_PIN);
			sensValue=sensValue<<1;
			LL_GPIO_ResetOutputPin(SENX_PORT, SEN2_CLK_PIN);
			if(LL_GPIO_IsInputPinSet(SENX_PORT, S2_DOUT_PIN))	
				{
					sensValue++;
				}
		}
	LL_GPIO_SetOutputPin(SENX_PORT, SEN2_CLK_PIN);
	sensValue=sensValue^0x800000;
	LL_GPIO_ResetOutputPin(SENX_PORT, SEN2_CLK_PIN);	
#endif	
	return(sensValue);	
}	

uint32_t readSenV(void)
{
	uint32_t sensValue=0;
#if 0	
	uint8_t i;
	
	LL_GPIO_ResetOutputPin(SENX_PORT, SEN3_CLK_PIN);
	while( LL_GPIO_IsInputPinSet(SENX_PORT, S3_DOUT_PIN));
	for(i=0;i<24;i++)
		{
			LL_GPIO_SetOutputPin(SENX_PORT, SEN3_CLK_PIN);
			sensValue=sensValue<<1;
			LL_GPIO_ResetOutputPin(SENX_PORT, SEN3_CLK_PIN);
			if(LL_GPIO_IsInputPinSet(SENX_PORT, S3_DOUT_PIN))	
				{
					sensValue++;
				}
		}
	LL_GPIO_SetOutputPin(SENX_PORT, SEN3_CLK_PIN);
	sensValue=sensValue^0x800000;
	LL_GPIO_ResetOutputPin(SENX_PORT, SEN3_CLK_PIN);
#endif	
	return(sensValue);	
}	

//key
//H in key dectech
uint8_t isKeyHinOn(void)
{
	uint8_t temp=0;
	if( LL_GPIO_IsInputPinSet(KEY_HIN_PORT,KEY_HIN_PIN) )
	{
		temp=0;	//high level,key=none
	}
	else
	{
		temp=1;
	}
	return temp;
}	
//H out key dectech	
uint8_t isKeyHoutOn(void)
{
	uint8_t temp=0;
	if( LL_GPIO_IsInputPinSet(KEY_HOT_PORT,KEY_HOT_PIN) )
	{
		temp=0;	//high level,key=none
	}
	else
	{
		temp=1;
	}
	return temp;
}	
//V up key dectech	
uint8_t isKeyVupOn(void)
{
	uint8_t temp=0;
	if( LL_GPIO_IsInputPinSet(KEY_VX_PORT,KEY_VUP_PIN ) )
	{
		temp=0;	//high level,key=none
	}
	else
	{
		temp=1;
	}
	return temp;
}	
//V up key dectech	
uint8_t isKeyVdownOn(void)
{
	uint8_t temp=0;
	if( LL_GPIO_IsInputPinSet(KEY_VX_PORT,KEY_VDN_PIN) )
	{
		temp=0;	//high level,key=none
	}
	else
	{
		temp=1;
	}
	return temp;
}


#if 0
//gpo init
//void gpoInit(void)
//{
//  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);  /* 使能时钟 */

//  LL_GPIO_InitTypeDef GPIO_InitStruct;
////PBx	
//  GPIO_InitStruct.Pin = SYS_HOLD_PIN  ;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT; 
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//	
//	sysHold();
//		
//  GPIO_InitStruct.Pin =  MOTO3_A_PIN ;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT; 
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//	
//  GPIO_InitStruct.Pin =  MOTO3_B_PIN ;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT; 
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//	
//  GPIO_InitStruct.Pin =  MOTO4_A_PIN;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT; 
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//	
//  GPIO_InitStruct.Pin = MOTO4_B_PIN ;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT; 
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//	
////PAx
//	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);  /* 使能时钟 */
//	
//  GPIO_InitStruct.Pin = MOTO1_A_PIN | MOTO1_B_PIN | MOTO2_A_PIN | MOTO2_B_PIN | SEN1_CLK_PIN | SEN2_CLK_PIN | SEN3_CLK_PIN | MOTO5_PIN ;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT; 
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//	
//	//...
//	
////PFx
//	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOF);  /* 使能时钟 */
//	
//  GPIO_InitStruct.Pin = LED_PIN  ;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT; 
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);
//	
//	ledOff();	
//}
#endif

//gpo init
void gpoInit(void)
{
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);  /* 使能时钟 */

  LL_GPIO_InitTypeDef GPIO_InitStruct;
//PBx	
  GPIO_InitStruct.Pin = SYS_HOLD_PIN | MOTO3_A_PIN | MOTO3_B_PIN | MOTO4_A_PIN | MOTO4_B_PIN ;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT; 
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	sysHold();
	
//PAx
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);  /* 使能时钟 */
	
  GPIO_InitStruct.Pin = MOTO1_A_PIN | MOTO1_B_PIN | MOTO2_A_PIN | MOTO2_B_PIN | SEN1_CLK_PIN | SEN2_CLK_PIN | SEN3_CLK_PIN | MOTO_VIBRATE_PIN ;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT; 
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
//motor off
	M1_OFF();
	mHoff();
	mVoff();
	mLoff();
	MOTO_VIBRATE_OFF();
	
	
//PFx
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOF);  /* 使能时钟 */
	
  GPIO_InitStruct.Pin = LED_PIN  ;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT; 
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);
	
	ledOff();	
}

//gpi init
void gpiInit(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct;	
//PAx
  GPIO_InitStruct.Pin = S1_DOUT_PIN | S2_DOUT_PIN | S3_DOUT_PIN  ;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT; 
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
	
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);	
	
  LL_GPIO_SetPinMode(GPIOA, KEY_HIN_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(GPIOA, KEY_HIN_PIN, LL_GPIO_PULL_UP);
  LL_GPIO_SetPinSpeed(GPIOA, KEY_HIN_PIN, LL_GPIO_SPEED_FREQ_LOW); 
	
//PBx
  GPIO_InitStruct.Pin = KEY_HOT_PIN | KEY_VUP_PIN | KEY_VDN_PIN  ;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT; 
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;

  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);		
}

//uart2 init
/**
  * @brief  USART配置函数
  * @param  USARTx：USART模块，可以是USART1、USART2
  * @retval 无
  */
static void APP_ConfigUsart(USART_TypeDef *USARTx)
{
  /*使能时钟、初始化引脚、使能NVIC中断*/
  if (USARTx == USART1) 
  {
    /*使能GPIOA时钟*/
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    /*使能USART1时钟*/
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);
    
    /*GPIOA配置*/
    LL_GPIO_InitTypeDef GPIO_InitStruct;
    /*选择2号引脚*/
    GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
    /*选择复用模式*/
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    /*选择输出速度*/
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    /*选择输出模式*/
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    /*选择上拉*/
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    /*复用为USART1功能*/
    GPIO_InitStruct.Alternate = LL_GPIO_AF1_USART1;
    /*GPIOA初始化*/
    LL_GPIO_Init(GPIOA,&GPIO_InitStruct);
    
    /*选择3号引脚*/
    GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
    /*复用为USART1功能*/
    GPIO_InitStruct.Alternate = LL_GPIO_AF1_USART1;
    /*GPIOA初始化*/
    LL_GPIO_Init(GPIOA,&GPIO_InitStruct);
    
    /*设置USART1中断优先级*/
    NVIC_SetPriority(USART1_IRQn,0);
    /*使能USART1中断*/
    NVIC_EnableIRQ(USART1_IRQn);
  }
  else if (USARTx == USART2)
  {
    /*使能GPIOF时钟*/
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOF);
    /*使能USART2时钟*/
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    
    /*GPIOF配置*/
    LL_GPIO_InitTypeDef GPIO_InitStruct;
    /*选择1号引脚*/
    GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
    /*选择复用模式*/
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    /*选择输出速度*/
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
    /*选择输出模式*/
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    /*选择上拉*/
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    /*复用为USART2功能*/
    GPIO_InitStruct.Alternate = LL_GPIO_AF4_USART2;
    /*GPIOA初始化*/
    LL_GPIO_Init(GPIOF,&GPIO_InitStruct);
    
    /*选择0号引脚*/
    GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
    /*复用为USART2功能*/
    GPIO_InitStruct.Alternate = LL_GPIO_AF4_USART2;
    /*GPIOA初始化*/
    LL_GPIO_Init(GPIOF,&GPIO_InitStruct);
    
    /*设置USART1中断优先级*/
    NVIC_SetPriority(USART2_IRQn,0);
    /*使能USART1中断*/
    NVIC_EnableIRQ(USART2_IRQn);
  }
  
  /*配置USART功能*/
  LL_USART_InitTypeDef USART_InitStruct;
  USART_InitStruct.BaudRate = 9600;//115200;
  /*设置数据长度*/
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;

  LL_USART_Init(USARTx, &USART_InitStruct);
  
  /*配置为全双工异步模式*/
  LL_USART_ConfigAsyncMode(USARTx);
  
  /*使能UART模块*/
  LL_USART_Enable(USARTx);
}

#endif

void uart2Init(void)
{
	APP_ConfigUsart(USART2);

  LL_USART_EnableIT_PE(USART2);		  	/*使能接收奇偶校验错误中断*/
  LL_USART_EnableIT_ERROR(USART2);		  /*使能接收错误中断*/
  LL_USART_EnableIT_RXNE(USART2);			  /*使能接收数据寄存器非空中断*/
}


/**
  * @brief  USART发送函数
  * @param  USARTx：USART模块，可以是USART1、USART2
  * @param  pData：发送缓冲区
  * @param  Size：发送缓冲区大小
  * @retval 无
  */
static void APP_UsartTransmit(USART_TypeDef *USARTx, uint8_t *pData, uint16_t Size)
{
  TxBuff = pData;
  TxCount = Size;
  
  /*发送数据*/
  while (TxCount > 0)
  {
    /* 等待TXE标志位置位 */
    while(LL_USART_IsActiveFlag_TXE(USARTx) != 1);
    /* 发送数据 */
    LL_USART_TransmitData8(USARTx, *TxBuff);
    TxBuff++;
    TxCount--;
  }
  
  /*等待TC标志位置位*/
  while(LL_USART_IsActiveFlag_TC(USARTx) != 1);
}

/**
  * @brief  USART接收函数
  * @param  USARTx：USART模块，可以是USART1、USART2
  * @param  pData：接收缓冲区
  * @param  Size：接收缓冲区大小
  * @retval 无
  */
static void APP_UsartReceive(USART_TypeDef *USARTx, uint8_t *pData, uint16_t Size)
{
  RxBuff = pData;
  RxCount = Size;
  
  /*接收数据*/
  while (RxCount > 0)
  {
    /* 等待RXNE标志位置位 */
    while(LL_USART_IsActiveFlag_RXNE(USARTx) != 1);
    /* 接收数据 */
    *RxBuff = LL_USART_ReceiveData8(USARTx);
    RxBuff++;
    RxCount--;
  }
}







#if 1
/**
  * @brief  USART发送函数
  * @param  USARTx：USART模块，可以是USART1、USART2
  * @param  pData：发送缓冲区
  * @param  Size：发送缓冲区大小
  * @retval 无
  */
static void APP_UsartTransmit_IT(USART_TypeDef *USARTx, uint8_t *pData, uint16_t Size)
{
  TxBuff = pData;
  TxSize = Size;
  TxCount = Size;
  
  /*使能发送数据寄存器空中断*/
  LL_USART_EnableIT_TXE(USARTx);
	NVIC_SetPriority(USART2_IRQn,0);
}

/**
  * @brief  USART接收函数
  * @param  USARTx：USART模块，可以是USART1、USART2
  * @param  pData：接收缓冲区
  * @param  Size：接收缓冲区大小
  * @retval 无
  */
static void APP_UsartReceive_IT(USART_TypeDef *USARTx, uint8_t *pData, uint16_t Size)
{
  RxBuff = pData;
  RxSize = Size;
  RxCount = Size;
  
  /*使能接收奇偶校验错误中断*/
  LL_USART_EnableIT_PE(USARTx);

  /*使能接收错误中断*/
  LL_USART_EnableIT_ERROR(USARTx);

  /*使能接收数据寄存器非空中断*/
  LL_USART_EnableIT_RXNE(USARTx);
	NVIC_SetPriority(USART2_IRQn,0);
}

/**
  * @brief  USART中断处理函数
  * @param  USARTx：USART模块，可以是USART1、USART2
  * @retval 无
  */
void APP_UsartIRQCallback(USART_TypeDef *USARTx)
{
  /* 接收数据寄存器不为空 */
  uint32_t errorflags = (LL_USART_IsActiveFlag_PE(USARTx) | LL_USART_IsActiveFlag_FE(USARTx) |\
                         LL_USART_IsActiveFlag_ORE(USARTx) | LL_USART_IsActiveFlag_NE(USARTx));
	
#if 0	
  if (errorflags == RESET)
  {
    if ((LL_USART_IsActiveFlag_RXNE(USARTx) != RESET) && (LL_USART_IsEnabledIT_RXNE(USARTx) != RESET))
    {
      *RxBuff = LL_USART_ReceiveData8(USARTx);
       RxBuff++;
      if (--RxCount == 0U)
      {
        LL_USART_DisableIT_RXNE(USARTx);
        LL_USART_DisableIT_PE(USARTx);
        LL_USART_DisableIT_ERROR(USARTx);
        
        UartReady = SET;
      }
      return;
    }
  }
  
  /* 接收错误 */ 
  if (errorflags != RESET)
  {
    APP_ErrorHandler();
  }



  
  /* 发送数据寄存器空 */ 
  if ((LL_USART_IsActiveFlag_TXE(USARTx) != RESET) && (LL_USART_IsEnabledIT_TXE(USARTx) != RESET))
  {
    LL_USART_TransmitData8(USARTx, *TxBuff);
    TxBuff++;
    if (--TxCount == 0U)
    { 
        LL_USART_DisableIT_TXE(USARTx);
        
        LL_USART_EnableIT_TC(USARTx);
    }
    return;
  }



  /* 发送完成 */
  if ((LL_USART_IsActiveFlag_TC(USARTx) != RESET) && (LL_USART_IsEnabledIT_TC(USARTx) != RESET))
  {
    LL_USART_DisableIT_TC(USARTx);
    UartReady = SET;
  
    return;
  }
	
#endif	
	

	
	LL_USART_DisableIT_RXNE(USART2);
	LL_USART_DisableIT_PE(USART2);
	LL_USART_DisableIT_ERROR(USART2);	

	aRxBuffer[0]=0;
	aRxBuffer[1]=0;
	aRxBuffer[2]=0;
	aRxBuffer[3]=0;
	aRxBuffer[4]=0;
	aRxBuffer[5]=0;
	aRxBuffer[6]=0;



	APP_UsartReceive(USART2, (uint8_t*)aRxBuffer, 7);	
	if( 0xFF==aRxBuffer[0]  )
	{
		if( 0x65==aRxBuffer[1] && 0x01==aRxBuffer[2] && 0xFF==aRxBuffer[5] && 0xFF==aRxBuffer[6] )
		{
			switch ( aRxBuffer[3] )
			{
				case 0x04:		//motor H in button
				{
					if( 1==aRxBuffer[4] )
					{
	#if 1					
						aTxBufDebug[1]='H';
						aTxBufDebug[2]='i';
						aTxBufDebug[3]='n';
						aTxBufDebug[4]='S';					
						APP_UsartTransmit	(USART2, (uint8_t*)aTxBufDebug, 8);
	#endif		
						flagMH=1;		//start motor H in		
					}			
					else if( 0==aRxBuffer[4] )
					{
	#if 1					
						aTxBufDebug[1]='H';
						aTxBufDebug[2]='i';
						aTxBufDebug[3]='n';
						aTxBufDebug[4]='E';
						APP_UsartTransmit	(USART2, (uint8_t*)aTxBufDebug, 8);
	#endif
						flagMH=0;
					}	

	//				aRxBuffer[0]=0;
	//				aRxBuffer[1]=0;
	//				aRxBuffer[2]=0;
	//				aRxBuffer[3]=0;
	//				aRxBuffer[4]=0;
	//				aRxBuffer[5]=0;
	//				aRxBuffer[6]=0;
					
					LL_USART_EnableIT_PE(USART2);		  	/*使能接收奇偶校验错误中断*/
					LL_USART_EnableIT_ERROR(USART2);		  /*使能接收错误中断*/
					LL_USART_EnableIT_RXNE(USART2);			  /*使能接收数据寄存器非空中断*/	

					break;				
				}
				case 0x05:		//motor H ouy button
				{
					if( 1==aRxBuffer[4] )
					{
	#if 1					
						aTxBufDebug[1]='H';
						aTxBufDebug[2]='o';
						aTxBufDebug[3]='t';
						aTxBufDebug[4]='S';
						APP_UsartTransmit	(USART2, (uint8_t*)aTxBufDebug, 8);
	#endif					
						flagMH=2;
					}			
					else if( 0==aRxBuffer[4] )
					{
	#if 1					
						aTxBufDebug[1]='H';
						aTxBufDebug[2]='o';
						aTxBufDebug[3]='t';
						aTxBufDebug[4]='E';
						APP_UsartTransmit	(USART2, (uint8_t*)aTxBufDebug, 8);
	#endif					
						flagMH=0;
					}		
					
	//				aRxBuffer[0]=0;
	//				aRxBuffer[1]=0;
	//				aRxBuffer[2]=0;
	//				aRxBuffer[3]=0;
	//				aRxBuffer[4]=0;
	//				aRxBuffer[5]=0;
	//				aRxBuffer[6]=0;
					
					LL_USART_EnableIT_PE(USART2);		  	/*使能接收奇偶校验错误中断*/
					LL_USART_EnableIT_ERROR(USART2);		  /*使能接收错误中断*/
					LL_USART_EnableIT_RXNE(USART2);			  /*使能接收数据寄存器非空中断*/	
					
					break;				
				}			
			}
			
		}
	}
	else if( 0x65==aRxBuffer[0] )
	{
		if( 0x01==aRxBuffer[1] && 0xFF==aRxBuffer[4] && 0xFF==aRxBuffer[5] && 0xFF==aRxBuffer[6] )
		{
			switch ( aRxBuffer[2] )
			{
				case 0x04:		//motor H in button
				{
					if( 1==aRxBuffer[3] )
					{
	#if 1					
						aTxBufDebug[1]='H';
						aTxBufDebug[2]='i';
						aTxBufDebug[3]='n';
						aTxBufDebug[4]='S';					
						APP_UsartTransmit	(USART2, (uint8_t*)aTxBufDebug, 8);
	#endif		
						flagMH=1;		//start motor H in		
					}			
					else if( 0==aRxBuffer[3] )
					{
	#if 1					
						aTxBufDebug[1]='H';
						aTxBufDebug[2]='i';
						aTxBufDebug[3]='n';
						aTxBufDebug[4]='E';
						APP_UsartTransmit	(USART2, (uint8_t*)aTxBufDebug, 8);
	#endif
						flagMH=0;
					}	

	//				aRxBuffer[0]=0;
	//				aRxBuffer[1]=0;
	//				aRxBuffer[2]=0;
	//				aRxBuffer[3]=0;
	//				aRxBuffer[4]=0;
	//				aRxBuffer[5]=0;
	//				aRxBuffer[6]=0;
					
					LL_USART_EnableIT_PE(USART2);		  	/*使能接收奇偶校验错误中断*/
					LL_USART_EnableIT_ERROR(USART2);		  /*使能接收错误中断*/
					LL_USART_EnableIT_RXNE(USART2);			  /*使能接收数据寄存器非空中断*/	

					break;				
				}
				case 0x05:		//motor H ouy button
				{
					if( 1==aRxBuffer[3] )
					{
	#if 1					
						aTxBufDebug[1]='H';
						aTxBufDebug[2]='o';
						aTxBufDebug[3]='t';
						aTxBufDebug[4]='S';
						APP_UsartTransmit	(USART2, (uint8_t*)aTxBufDebug, 8);
	#endif					
						flagMH=2;
					}			
					else if( 0==aRxBuffer[3] )
					{
	#if 1					
						aTxBufDebug[1]='H';
						aTxBufDebug[2]='o';
						aTxBufDebug[3]='t';
						aTxBufDebug[4]='E';
						APP_UsartTransmit	(USART2, (uint8_t*)aTxBufDebug, 8);
	#endif					
						flagMH=0;
					}		
					
	//				aRxBuffer[0]=0;
	//				aRxBuffer[1]=0;
	//				aRxBuffer[2]=0;
	//				aRxBuffer[3]=0;
	//				aRxBuffer[4]=0;
	//				aRxBuffer[5]=0;
	//				aRxBuffer[6]=0;
					
					LL_USART_EnableIT_PE(USART2);		  	/*使能接收奇偶校验错误中断*/
					LL_USART_EnableIT_ERROR(USART2);		  /*使能接收错误中断*/
					LL_USART_EnableIT_RXNE(USART2);			  /*使能接收数据寄存器非空中断*/	
					
					break;				
				}			
			}
			
		}
	}
	else
	{
#if 1					
		aTxBufDebug[1]='e';
		aTxBufDebug[2]='r';
		aTxBufDebug[3]='r';
		aTxBufDebug[4]='!';					
		APP_UsartTransmit	(USART2, (uint8_t*)aTxBufDebug, 8);
		
		LL_USART_EnableIT_PE(USART2);		  	/*使能接收奇偶校验错误中断*/
		LL_USART_EnableIT_ERROR(USART2);		  /*使能接收错误中断*/
		LL_USART_EnableIT_RXNE(USART2);			  /*使能接收数据寄存器非空中断*/			
#endif	
	}
}


#endif
//timer1 init->1ms
/*
	->	8kk(defalut hri freq)/prescaler(2k)=timerx clk freq	:	8kk/2k=4k
	->	auroreload(2k)*(1/timer clk freq)=timing value:				2k*(1/4k)=0.5s
*/
void timer1Init(void)
{
  LL_APB1_GRP2_EnableClock(RCC_APBENR2_TIM1EN);	  /* 使能TIM1时钟 */
	
  LL_TIM_InitTypeDef TIM1CountInit = {0};	  /* 配置TIM1 */
  TIM1CountInit.ClockDivision       = LL_TIM_CLOCKDIVISION_DIV1; /* 时钟不分频 */
  TIM1CountInit.CounterMode         = LL_TIM_COUNTERMODE_UP;     /* 向上计数模式 */
  TIM1CountInit.Prescaler           = 1000-1;                    // 预分频值：1000 -> clk freq=8kk/1k=8k
  TIM1CountInit.Autoreload          = 8-1;                    // 自动重装载值：8-1 -> timig=8*(1/8k)=1ms
  TIM1CountInit.RepetitionCounter   = 0;                         /* 重复计数值：0 */
  
  LL_TIM_Init(TIM1,&TIM1CountInit);	  /* 初始化TIM1 */
  
  /* 使能UPDATA中断 */
  LL_TIM_EnableIT_UPDATE(TIM1);
  
  /* 使能TIM1计数器 */
  LL_TIM_EnableCounter(TIM1);
  
  /* 开启UPDATA中断请求 */
  NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
  NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn,1);
}

/**
  * @brief  TIM更新中断回调函数
  * @param  无
  * @retval 无
  */
void timer1UpdateCallback(void)
{
//	if( isKeyHinOn() || isKeyHoutOn() )
//	{
//		mHoff();
//	}
//	if( isKeyVupOn() || isKeyVdownOn() )
//	{
//		mVoff();
//	}
	if(senxRefresh++>499)
	{
		senxRefresh=0;
#if 0		
		//flgmH=?ffffff
		aTxBufDebug2[6]=0x30+flagMH;
		APP_UsartTransmit	(USART2, (uint8_t*)aTxBufDebug2, 10);
#endif		
		
		
		
		
		
		aTxBuffer[0]='x';
		aTxBuffer[2]='.';
		aTxBuffer[3]='v';
		aTxBuffer[4]='a';
		aTxBuffer[5]='l';
		aTxBuffer[6]='=';
		
		//ledToggle();		//翻转LED
		//tToggle();		//翻转LED,demo board only!!!
		
		valSenH+=10;
		aTxBuffer[1]='0';
		aTxBuffer[7]=0x30 + valSenH/1000 ;
		aTxBuffer[8]=0x30 + valSenH%1000/100 ;
		aTxBuffer[9]=0x30 + valSenH%10 ;
			
		APP_UsartTransmit	(USART2, (uint8_t*)aTxBuffer, 13);
//		LL_mDelay(100);	
			
		valSenV+=22;
		aTxBuffer[1]='1';
		if(valSenV/1000 > 9)
		{
			aTxBuffer[7]=0x30;
		}
		else
		{
			aTxBuffer[7]=0x30 + valSenV/1000 ;
		}

		if(valSenV%1000/100 > 9)
		{
			aTxBuffer[8]=0x31;
		}
		else
		{
			aTxBuffer[8]=0x30 + valSenV%1000/100 ;
		}	
		if(valSenV%10 > 9)
		{
			aTxBuffer[9]=0x33;
		}
		else
		{
			aTxBuffer[9]=0x30 + valSenV%10 ;
		}		


		APP_UsartTransmit	(USART2, (uint8_t*)aTxBuffer, 13);
//		LL_mDelay(100);			
	}
}


/* Private function end-------------------------------------------------------*/




/**
  * @brief  应用程序入口函数.
  * @param  无
  * @retval int
  */
int main(void)
{
	sysInit();
	gpoInit();
	gpiInit();
	
	uart2Init();
	
	timer1Init();

	aTxBuffer[10] = 0xFF;
	aTxBuffer[11] = 0xFF;
	aTxBuffer[12] = 0xFF;
	
	aTxBufDebug[0]='m';
	aTxBufDebug[5]=0xFF;
	aTxBufDebug[6]=0xFF;
	aTxBufDebug[7]=0xFF;	
	
	aTxBufDebug2[7]=0xFF;
	aTxBufDebug2[8]=0xFF;
	aTxBufDebug2[9]=0xFF;	

//M1->NONE->H
//M2->H->NONE
//M3->V
//M4->ROTATE
#if 0
	while(1)
	{
		ledOn();
		M1_FF();				//in
		LL_mDelay(1000);
		ledOff();
		M1_OFF();
		LL_mDelay(100);
		ledOn();
		M1_FW();				//out
		LL_mDelay(1000);
		ledOff();
		M1_OFF();
		LL_mDelay(100);		
		
//		mHin();				//in
//		LL_mDelay(10000);
//		mHoff();
//		LL_mDelay(100);
//		mHout();				//out
//		LL_mDelay(10000);
//		mHoff();
//		LL_mDelay(100);
		ledOn();
		mVup();
		LL_mDelay(1000);
		ledOff();
		mVoff();
		LL_mDelay(100);
		ledOn();
		mVdown();
		LL_mDelay(1000);
		ledOff();
		mVoff();
		LL_mDelay(100);

		ledOn();
		mLup();
		LL_mDelay(1000);
		ledOff();
		mLoff();
		LL_mDelay(100);
		ledOn();
		mLdown();
		LL_mDelay(1000);
		ledOff();
		mLoff();
		LL_mDelay(100);

		ledOn();
		MOTO_VIBRATE_ON();
		LL_mDelay(1000);
		ledOff();
		MOTO_VIBRATE_OFF();		
		LL_mDelay(100);
		
		
	}
#endif


	while( flagSys )
	{
		if( runMode == 2 )	//auto mode
		{
			if(!flagReadSetting)		//setting value read once
			{
				flagReadSetting=1;
				//read saved H and V value 
			}
			
			while( valSenH != valSenHsaved )		//motor horizontal
			{
				if(	readSenH() < valSenHsaved )
				{
					mHin();
				}
				else if( readSenH() > valSenHsaved )
				{
					mHout();
				}
			}
			
			while( valSenV != valSenVsaved )		//motor vertical
			{
				if(	readSenV() < valSenVsaved )
				{
					mVup();
				}
				else if( readSenH() > valSenVsaved )
				{
					mVdown();
				}
			}

		}
		else	//manual or setting mode or runMode error
		{
			if( flagMH )		//motor horizontal
			{
				//if(flagMH==1 && !isKeyHinOn() )
				if(flagMH==1  )	
				{
					//mHin();
					M1_FF();
				}
				//else if(flagMH==2 && !isKeyHoutOn() )
				else if(flagMH==2 )
				{
					//mHout();
					M1_FW();
				}
				else
				{
					//mHoff();
					M1_OFF();
				}
			}
			else
			{
				//mHoff();
				M1_OFF();
			}

			if( flagMV )		//motor vertical
			{
				if(flagMV==1 && !isKeyVupOn() )
				{
					mVup();
				}
				else if(flagMH==2 && !isKeyVdownOn() )
				{
					mVdown();
				}
				else
				{
					mVoff();
				}
			}
			else
			{
				mVoff();
			}
			
			if( flagML )		//motor lean
			{
				if(flagML==1  )
				{
					mLup();
				}
				else if(flagMH==2  )
				{
					mLdown();
				}
				else
				{
					mLoff();
				}
			}
			else
			{
				mLoff();
			}
			
			if( readSenS() < 0 )		//is somebody on seat?
			{
				mHoff();
				mVoff();
				mLoff();
				flagAlarm=0;
				flagSys=0;
			}			
		}
		
	if( readSenS() < 0 )		//is somebody on seat?
	{
		if(flagSys)
		{
			flagSys=0;
		}
	}
	else
	{
		flagSys=1;
	}
}

}



/**
  * @brief  错误执行函数
  * @param  无
  * @retval 无
  */
void APP_ErrorHandler(void)
{
  /* 无限循环 */
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  输出产生断言错误的源文件名及行号
  * @param  file：源文件名指针
  * @param  line：发生断言错误的行号
  * @retval 无
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* 用户可以根据需要添加自己的打印信息,
     例如: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* 无限循环 */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
