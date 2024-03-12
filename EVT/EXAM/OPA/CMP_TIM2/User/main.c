/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/12/26
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*
 *@Note
 * CMP1_P1--PA0
 * CMP1_N0--PC3
 * CMP1_OUT--TIM2_CH1
 * In this example, PA0 and PC3 are postive and negative input, the outport remap to TIM2_CH1.
 * The output level of the comparator is used for PWM capture of the timer.
 */

#include "debug.h"


/*********************************************************************
 * @fn      CMP_Init
 *
 * @brief   Initializes CMP.
 *
 * @return  none
 */
void CMP_Init( void )
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    CMP_InitTypeDef  CMP_InitTypeDef = {0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC, ENABLE );
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init( GPIOC, &GPIO_InitStructure );
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init( GPIOA, &GPIO_InitStructure );

    CMP_InitTypeDef.CMP_NUM=CMP1;
    CMP_InitTypeDef.Mode = OUT_IO_TIM2;
    CMP_InitTypeDef.NSEL = CMP_CHN0;
    CMP_InitTypeDef.PSEL=CMP_CHP2;
    CMP_InitTypeDef.HYEN=CMP_HYEN2;
    OPA_CMP_Init(&CMP_InitTypeDef);
    OPA_CMP_Cmd(CMP1,ENABLE);
}


/*********************************************************************
 * @fn      Input_Capture_Init
 *
 * @brief   Initializes TIM2 input capture.
 *
 * @param   arr - the period value.
 *          psc - the prescaler value.
 *          ccp - the pulse value.
 *
 * @return  none
 */
void Input_Capture_Init(u16 arr, u16 psc)
{

    TIM_ICInitTypeDef       TIM_ICInitStructure = {0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};
    NVIC_InitTypeDef        NVIC_InitStructure = {0};

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0x00;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x00;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;

    TIM_PWMIConfig(TIM2, &TIM_ICInitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = TIM2_CC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ITConfig(TIM2, TIM_IT_CC1 | TIM_IT_CC2, ENABLE);

    TIM_SelectInputTrigger(TIM2, TIM_TS_TI1FP1);
    TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);
    TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);
    TIM_Cmd(TIM2, ENABLE);
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
    Delay_Init();
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );
    OPA_CMP_Unlock();
    CMP_Init();
    Input_Capture_Init(0XFFFF,4800);
    while(1);
}

void TIM2_CC_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      TIM2_CC_IRQHandler
 *
 * @brief   This function handles TIM2  Capture Compare Interrupt exception.
 *
 * @return  none
 */
void TIM2_CC_IRQHandler(void)
{
    if( TIM_GetITStatus( TIM2, TIM_IT_CC1 ) != RESET )
    {
        printf( "CH1_Val:%d\r\n", TIM_GetCapture1( TIM2 ) );
        TIM_SetCounter( TIM2, 0 );
    }

    if( TIM_GetITStatus( TIM2, TIM_IT_CC2 ) != RESET )
    {
        printf( "CH2_Val:%d\r\n", TIM_GetCapture2( TIM2 ) );
    }

    TIM_ClearITPendingBit( TIM2, TIM_IT_CC1 | TIM_IT_CC2 );
}




