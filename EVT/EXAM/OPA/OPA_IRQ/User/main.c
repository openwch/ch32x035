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
 *OPA interrupt routine:
 *OPA2_CHP0--PA7
 *OPA2_CHN1--PA5
 *OPA2_OUT--PA4
 *
 *In this example, PA5 and PA4 are short-circuited, and the external voltage is input from PA7,
 *when PA7 high voltage enter the OPA interrupt when the OPA in query function
 */

#include "debug.h"


/*********************************************************************
 * @fn      OPA2_Init
 *
 * @brief   Initializes OPA2.
 *
 * @return  none
 */
void OPA2_Init( void )
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    OPA_InitTypeDef  OPA_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE );
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_5|GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init( GPIOA, &GPIO_InitStructure );

    NVIC_InitStructure.NVIC_IRQChannel = OPA_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    OPA_InitStructure.OPA_NUM=OPA2;
    OPA_InitStructure.NSEL = CHN0;
    OPA_InitStructure.PSEL = CHP0;
    OPA_InitStructure.POLL_NUM=CHP_POLL_NUM_3;
    OPA_InitStructure.PSEL_POLL=CHP_OPA1_ON_OPA2_ON;
    OPA_InitStructure.OPA_POLL_Interval=0x50;
    OPA_InitStructure.Mode=OUT_IO_OUT0;
    OPA_InitStructure.OUT_IE=OUT_IE_OPA1_ON_OPA2_ON;

    OPA_Init( &OPA_InitStructure );
    OPA_Cmd(OPA2, ENABLE );
}

void OPA_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      OPA_IRQHandler
 *
 * @brief   This function handles OPA exception.
 *
 * @return  none
 */
void OPA_IRQHandler(void)
{
    if(OPA_GetFlagStatus(OPA_FLAG_OUT_OPA2)!=RESET)
    {
    printf("Run at EXTI\r\n");
    OPA_ClearFlag(OPA_FLAG_OUT_OPA2);     /* Clear Flag */
    }
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
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );
    OPA_Unlock();
    OPA_POLL_Unlock();
    OPA2_Init();

    while(1)
    {
    }
}

