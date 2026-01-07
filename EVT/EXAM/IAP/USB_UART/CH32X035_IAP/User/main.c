/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.2
 * Date               : 2025/10/27
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*
 * IAP routine: this routine support USB and UART mode,
 * and you can choose the command method or the IO method to jump to the APP .
 * Key  parameters: CalAddr - address in flash (same in APP), note that this address needs to be unused.
 *                  CheckNum - The value of 'CalAddr' that needs to be modified.
 * Tips :the routine need IAP software version 1.50 or later.
 */

#include "ch32x035_usbfs_device.h"
#include "debug.h"
#include "iap.h"

extern vu8 End_Flag;

#define UPGRADE_MODE_COMMAND   0
#define UPGRADE_MODE_IO        1

#define UPGRADE_MODE   UPGRADE_MODE_COMMAND
/*********************************************************************
 * @fn      IAP_2_APP
 *
 * @brief   IAP_2_APP program.
 *
 * @return  none
 */
void IAP_2_APP(void) {
    USBFS_Device_Init(DISABLE,PWR_VDD_SupplyVoltage());
    GPIO_Cfg_Float();
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, DISABLE );
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_USBFS, DISABLE );
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,DISABLE);

    Delay_Ms(50);
    printf("jump APP\r\n");
    Delay_Ms(10);
    NVIC_EnableIRQ(Software_IRQn);
    NVIC_SetPendingIRQ(Software_IRQn);
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
    printf("ChipID:%08x\r\n", DBGMCU_GetCHIPID() );
    printf("IAP\r\n");
#if UPGRADE_MODE == UPGRADE_MODE_COMMAND
    if(*(uint32_t*)FLASH_Base  != 0xFFFFFFFF )
    {
        if(*(uint32_t*)CalAddr != CheckNum)
        {
            IAP_2_APP();
            while(1);
        }
    }
#elif UPGRADE_MODE == UPGRADE_MODE_IO
    if(PA0_Check() == 0)
    {
        IAP_2_APP();
        while(1);
    }
#endif
    /* Usb Init */
    USBFS_RCC_Init( );
    USBFS_Device_Init( ENABLE ,PWR_VDD_SupplyVoltage());

    USART2_CFG(460800);
    while(1)
    {
        if( USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET) {
            UART_Rx_Deal();
        }
#if UPGRADE_MODE == UPGRADE_MODE_COMMAND
        if (End_Flag)
        {
            Delay_Ms(10);
            IAP_2_APP();
            while(1);
        }
#endif
    }
}
