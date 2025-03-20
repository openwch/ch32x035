/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2025/03/11
 * Description        : Main program body.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/*
 * @Note
 * This routine is used to demonstrate the CH211 Demo board.Users can select
 * the role of the Demo board through the Set_DevChk(), thereby verifying
 * the related functions of the PD and CH211 chip.
 *
 * Schematic Path: EVT/PUB/SCHPCB/CH32X035USBPD_CH211/USBPD_CH211.SchDoc
 */

#include <CH211_I2C.h>
#include "string.h"
#include "debug.h"
#include "libCH32_USBPD.h"
#include "PD_User.h"
#include "PD_Prot.h"

/*********************************************************************
 * @fn      LED_Init
 *
 * @brief   Initializes the LED.
 *
 * @return  none
 */
void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure = {0};
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_SetBits(GPIOA, GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4 );		//LED RGB
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/*********************************************************************
 * @fn      ADC_Function_Init
 *
 * @brief   Initializes the ADC.
 *
 * @return  none
 */
void ADC_Function_Init(void)
{
    ADC_InitTypeDef  ADC_InitStructure = {0};
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    DMA_InitTypeDef DMA_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_ADC1  , ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    ADC_DeInit(ADC1);

    ADC_CLKConfig(ADC1, ADC_CLK_Div6);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);

     DMA_DeInit(DMA1_Channel1);
     DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->RDATAR;
     DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&GetADC_VBUS;
     DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
     DMA_InitStructure.DMA_BufferSize = 1;
     DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
     DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
     DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
     DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
     DMA_InitStructure.DMA_Mode = DMA_Mode_Circular ;
     DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
     DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
     DMA_Init(DMA1_Channel1, &DMA_InitStructure);

     DMA_Cmd(DMA1_Channel1, ENABLE);
     ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_11Cycles);
     ADC_SoftwareStartConvCmd(ADC1, ENABLE);

     vSafe0V =151;              //0.8V
     vSinkDisconnect = 693;     //3.67V
     vVBUSon =    429;
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
	SystemCoreClockUpdate();
	USART_Printf_Init(460800);
	printf("SystemClk:%d\r\n",SystemCoreClock);
	printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );

	Delay_Init();
	Delay_Ms(500);

	LED_Init();
    ADC_Function_Init();                    //VBUS voltage detection setting
	I2C_PWR_Init();							//I2C and CH211 initialization
	PD_Init();      						//PD initialization
	Set_DevChk( DevRole_Sink );				//Start detecting device plug and unplug

	while (1)
	{
		if ( PD_DEVICE.ConnectStat == 0 )
		{
			GPIO_ResetBits(GPIOA, LED_B );
            Delay_Ms(500);
			GPIO_SetBits(GPIOA, LED_B );
            Delay_Ms(500);
		}
		else
		{
			GPIO_SetBits(GPIOA, LED_B );
            Delay_Ms(500);
        }
	}
}

