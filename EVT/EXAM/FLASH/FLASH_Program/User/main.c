/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/03/22
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*
 *@Note
 *FLASH erase/read/write, fast programming and OptionBytes programming:
 *Includes Standard Erase and Program, Fast Erase and Program.
 *
*/

#include "debug.h"


/* Global define */
u32 buf[64];

/*********************************************************************
 * @fn      Option_Byte_CFG
 *
 * @brief   Config Option byte and enable reset pin.
 *
 * @return  none
 */
void Option_Byte_CFG(void)
{
    FLASH_Status status = FLASH_COMPLETE;

    status = FLASH_UserOptionByteConfig(OB_IWDG_SW, OB_STOP_NoRST, OB_STDBY_NoRST, OB_RST_EN_DT12ms);

    if(status == FLASH_RDP) printf("MCU in read protected state, user option byte cannot be configured. \
            Need to use WCHISPTool or WCH-LinkUtility for configuration ");
    else{
        printf("0x1FFFF800-%08x\r\n", *(u32*)0x1FFFF800);
        printf("0x1FFFF804-%08x\r\n", *(u32*)0x1FFFF804);
        printf("0x1FFFF808-%08x\r\n", *(u32*)0x1FFFF808);
        printf("0x1FFFF80C-%08x\r\n", *(u32*)0x1FFFF80C);
    }
}

/*********************************************************************
 * @fn      Flash_Test_Fast
 *
 * @brief   Flash Fast Program Test.
 *
 * @return  none
 */
void Flash_Test_Fast(void)
{
    u8  i, Verity_Flag = 0;

    for(i = 0; i < 64; i++){
        buf[i] = i;
    }

    FLASH_Unlock_Fast();

    FLASH_ErasePage_Fast(0x08003000);

    printf("256Byte Page Erase Suc\r\n");

    FLASH_BufReset();
    for(i=0; i<64; i++){
        FLASH_BufLoad(0x08003000+4*i, buf[i]);
    }

    FLASH_ProgramPage_Fast(0x08003000);

    printf("256Byte Page Program Suc\r\n");

    FLASH_Lock_Fast();

    for(i = 0; i < 64; i++){
        if(buf[i] == *(u32 *)(0x08003000 + 4 * i))
        {
            Verity_Flag = 0;
        }
        else
        {
            Verity_Flag = 1;
            break;
        }
    }

    if(Verity_Flag)
        printf("256Byte Page Verity Fail\r\n");
    else
        printf("256Byte Page Verity Suc\r\n");
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

    Delay_Ms(1000);
    USART_Printf_Init(115200);
    printf("SystemClk-1:%d\r\n", SystemCoreClock);
    printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );
    Flash_Test_Fast();
    Option_Byte_CFG();

    while(1);
}


