/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/04/06
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*
 *@Note
 *OPA reset routune:
 *OPA output terminal is high level, OPA is reset.
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

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE );
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_7|GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init( GPIOA, &GPIO_InitStructure );

    OPA_InitStructure.OPA_NUM=OPA2;
    OPA_InitStructure.NSEL = CHN0;
    OPA_InitStructure.PSEL = CHP0;
    OPA_InitStructure.RST_EN=RST_OPA1_OFF_OPA2_ON;
    OPA_InitStructure.PSEL_POLL=CHP_OPA1_OFF_OPA2_ON;
    OPA_InitStructure.OPA_POLL_Interval=0x50;
    OPA_InitStructure.Mode=OUT_IO_OUT0;
    OPA_Init( &OPA_InitStructure );
    OPA_Cmd(OPA2, ENABLE );
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
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );
    printf( "RCC->RSTSCKR:%08x\r\n", RCC->RSTSCKR );

    RCC->RSTSCKR |= (1<<24);
    RCC->RSTSCKR &= ~(1<<24);

    OPA_POLL_Unlock();
    OPA_Unlock();
    OPA2_Init();

    while(1);
}
