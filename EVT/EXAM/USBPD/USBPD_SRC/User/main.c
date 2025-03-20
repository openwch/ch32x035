/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.1
 * Date               : 2025/03/11
 * Description        : Main program body.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/*
 *@Note
 *
 * PD SRC Sample code
 *
 * This sample code may have compatibility issues and is for learning purposes only.
 *
 * Be sure to remove the pull-down resistors on both CC wires when using this Sample code!
 *
 * The inability to control the VBUS voltage on the board may lead to some compatibility problems,
 * mainly manifested in the inability of some devices to complete the PD communication process.
 *
 */

#include "debug.h"
#include "PD_Process.h"

void TIM1_UP_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
#if(Wake_up_mode==USBPDWake_up)
void USBPDWakeUp_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
#elif(Wake_up_mode==GPIOWake_up)
void EXTI15_8_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
#endif
volatile UINT8  Tim_Ms_Cnt = 0x00;

/*********************************************************************
 * @fn      TIM1_Init
 *
 * @brief   Initialize TIM1
 *
 * @return  none
 */
void TIM1_Init( u16 arr, u16 psc )
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};
    NVIC_InitTypeDef NVIC_InitStructure={0};
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_TIM1, ENABLE );
    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0x00;
    TIM_TimeBaseInit( TIM1, &TIM_TimeBaseInitStructure);
    TIM_ClearITPendingBit( TIM1, TIM_IT_Update );
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    TIM_ITConfig( TIM1, TIM_IT_Update , ENABLE );
    TIM_Cmd( TIM1, ENABLE );
}

/*********************************************************************
 * @fn      EXTI_INIT
 *
 * @brief   Initialize Wake up EXTI
 *
 * @return  none
 */
void EXTI_INIT(void)
{
#if(Wake_up_mode==USBPDWake_up)
    EXTI_InitTypeDef EXTI_InitStructure = {0};
    EXTI_InitStructure.EXTI_Line = EXTI_Line29;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
#elif(Wake_up_mode==GPIOWake_up)
    EXTI_InitTypeDef EXTI_InitStructure = {0};
    /* GPIOC.14 ----> EXTI_Line14 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource14);
    EXTI_InitStructure.EXTI_Line = EXTI_Line14;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* GPIOC.15 ----> EXTI_Line15 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource15);
    EXTI_InitStructure.EXTI_Line = EXTI_Line15;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
#endif

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
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(921600);
    printf( "SystemClk:%d\r\n", SystemCoreClock );
    printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );
    printf( "PD SRC TEST\r\n" );
    PD_Init( );
    EXTI_INIT();
    TIM1_Init( 999, 48-1);
    while(1)
    {
        /* Get the calculated timing interval value */
        TIM_ITConfig( TIM1, TIM_IT_Update , DISABLE );
        Tmr_Ms_Dlt = Tim_Ms_Cnt - Tmr_Ms_Cnt_Last;
        Tmr_Ms_Cnt_Last = Tim_Ms_Cnt;
        TIM_ITConfig( TIM1, TIM_IT_Update , ENABLE );
        PD_Ctl.Det_Timer += Tmr_Ms_Dlt;
        if( PD_Ctl.Det_Timer > 4 )
        {
            PD_Ctl.Det_Timer = 0;
            PD_Det_Proc( );
        }
        PD_Main_Proc( );
    }
}

/*********************************************************************
 * @fn      TIM1_UP_IRQHandler
 *
 * @brief   This function handles TIM1 interrupt.
 *
 * @return  none
 */
void TIM1_UP_IRQHandler(void)
{
    if( TIM_GetITStatus( TIM1, TIM_IT_Update ) != RESET )
    {
        Tim_Ms_Cnt++;
        TIM_ClearITPendingBit( TIM1, TIM_IT_Update );
    }
}

/*********************************************************************
 * @fn      EXTI15_8_IRQHandler
 *
 * @brief   This function handles EXTI14 and EXTI15 exception.
 *
 * @return  none
 */
void EXTI15_8_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line14)!=RESET)
  {
      SystemInit();
      printf(" GPIO Wake_up\r\n");
      EXTI_ClearITPendingBit(EXTI_Line14);     /* Clear Flag */
      NVIC_DisableIRQ(EXTI15_8_IRQn);
  }
  if(EXTI_GetITStatus(EXTI_Line15)!=RESET)
  {
      SystemInit();
      printf(" GPIO Wake_up\r\n");
      EXTI_ClearITPendingBit(EXTI_Line15);     /* Clear Flag */
      NVIC_DisableIRQ(EXTI15_8_IRQn);
    }

}

/*********************************************************************
 * @fn      USBPDWakeUp_IRQHandler
 *
 * @brief   This function handles USBPD WakeUp exception.
 *
 * @return  none
 */
void USBPDWakeUp_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line29)!=RESET)
  {
      SystemInit();
      EXTI_ClearITPendingBit(EXTI_Line29);     /* Clear Flag */
      NVIC_DisableIRQ(USBPDWakeUp_IRQn);
      USBPD->CONFIG&=~IE_PD_IO;
      printf("USBPDWakeUp\r\n");
      USBPD->PORT_CC1&=~(CC_PU_Mask);
      USBPD->PORT_CC2&=~(CC_PU_Mask);
      USBPD->PORT_CC1|=CC_PU_330;
      USBPD->PORT_CC2|=CC_PU_330;
  }
}

