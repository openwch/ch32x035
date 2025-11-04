/********************************** (C) COPYRIGHT  *******************************
 * File Name          : iap.c
 * Author             : WCH
 * Version            : V1.0.2
 * Date               : 2025/10/27
 * Description        : IAP
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "iap.h"
#include "string.h"
#include "core_riscv.h"

/******************************************************************************/

iapfun jump2app;
vu32 Program_addr = FLASH_Base;
vu32 Verify_addr = FLASH_Base;
u32 User_APP_Addr_offset = 0x5000;
vu8 Verify_Star_flag = 0;
u8 Fast_Program_Buf[390];
u32 Program_Buf[64];
vu32 CodeLen = 0;
vu8 End_Flag = 0;
u8 EP2_Rx_Buffer[USBD_DATA_SIZE+4];
#define  isp_cmd_t   ((isp_cmd  *)EP2_Rx_Buffer)

/*********************************************************************
 * @fn      CH32_IAP_Program
 *
 * @brief   adr - the date address
 *          buf - the date buffer
 *
 * @return  none
 */
void CH32_IAP_Program(u32 adr, u32* buf)
{
    u8 i;

    FLASH_BufReset();
    for(i=0; i<64; i++){
        FLASH_BufLoad(adr+4*i, buf[i]);
    }
    FLASH_ProgramPage_Fast(adr);
}

/*********************************************************************
 * @fn      Program_Buf_Modify
 *
 * @brief   Program_Buf Modify
 *
 * @return  none
 */
void Program_Buf_Modify(void)
{
    for(int i = 0; i <64; i++)
    {
        if(((CalAddr & 0xFFFFFF00)+4*i) != (uint32_t)CalAddr)
        {
            Program_Buf[i] = *(uint32_t*)((CalAddr & 0xFFFFFF00)+4*i);
        }
        else
        {
            Program_Buf[i] = (uint32_t)CheckNum;
        }
    }
}

/*********************************************************************
 * @fn      RecData_Deal
 *
 * @brief   UART-USB deal data(deal jump IAP command)
 *
 * @return  ERR_ERROR - ERROR
 *          ERR_SUCCESS - SUCCESS
 *          ERR_End - End
 */
u8 RecData_Deal(void)
{
     u8 s;

     switch ( isp_cmd_t->other.buf[0]) {
     case CMD_IAP_ERASE:
         s = ERR_ERROR;
         break;

     case CMD_IAP_PROM:
         s = ERR_ERROR;
         break;

     case CMD_IAP_VERIFY:
         s = ERR_ERROR;
         break;

     case CMD_IAP_END:
         s = ERR_ERROR;
         break;

     case CMD_JUMP_IAP:
         FLASH_Unlock_Fast();
         Program_Buf_Modify();
         FLASH_ErasePage_Fast(CalAddr & 0xFFFFFF00);
         CH32_IAP_Program(CalAddr & 0xFFFFFF00,(u32*)Program_Buf);
         FLASH->CTLR |= ((uint32_t)0x00008000);
         FLASH->CTLR |= ((uint32_t)0x00000080);

         s = ERR_SUCCESS;
         break;
     default:
         s = ERR_ERROR;
         break;
     }

     return s;
}

/*********************************************************************
 * @fn      USART2_CFG
 *
 * @brief   baudrate:UART2 baudrate
 *
 * @return  none
 */
void USART2_CFG(u32 baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl =
    USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART2, &USART_InitStructure);
    USART_Cmd(USART2, ENABLE);
}

/*********************************************************************
 * @fn      UART2_SendData
 *
 * @brief   USART2 send date
 *
 * @param   pbuf - Packet to be sent
 *          num - the number of date
 *
 * @return  none
 */
void UART2_SendData(u8 data)
{
    while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
    USART_SendData(USART2, data);
}

/*********************************************************************
 * @fn      Uart2_Rx
 *
 * @brief   Uart2 receive date
 *
 * @return  none
 */
u8 Uart2_Rx(void)
{
    while( USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);
    return USART_ReceiveData( USART2);
}

/*********************************************************************
 * @fn      UART_Rx_Deal
 *
 * @brief   UART Rx data deal
 *
 * @return  none
 */
void UART_Rx_Deal(void)
{
    u8 i, s;
    u16 Data_add = 0;

    if (Uart2_Rx() == Uart_Sync_Head1)
    {
        if (Uart2_Rx() == Uart_Sync_Head2)
        {
            isp_cmd_t->UART.Cmd = Uart2_Rx();
            Data_add += isp_cmd_t->UART.Cmd;
            isp_cmd_t->UART.Len = Uart2_Rx();
            Data_add += isp_cmd_t->UART.Len;

            if(isp_cmd_t->UART.Cmd == CMD_IAP_ERASE ||isp_cmd_t->UART.Cmd == CMD_IAP_VERIFY)
            {
                isp_cmd_t->other.buf[2] = Uart2_Rx();
                Data_add += isp_cmd_t->other.buf[2];
                isp_cmd_t->other.buf[3] = Uart2_Rx();
                Data_add += isp_cmd_t->other.buf[3];
                isp_cmd_t->other.buf[4] = Uart2_Rx();
                Data_add += isp_cmd_t->other.buf[4];
                isp_cmd_t->other.buf[5] = Uart2_Rx();
                Data_add += isp_cmd_t->other.buf[5];
            }
            if ((isp_cmd_t->other.buf[0] == CMD_IAP_PROM) || (isp_cmd_t->other.buf[0] == CMD_IAP_VERIFY))
            {
                for (i = 0; i < isp_cmd_t->UART.Len; i++) {
                    isp_cmd_t->UART.data[i] = Uart2_Rx();
                    Data_add += isp_cmd_t->UART.data[i];
                }
            }
            if (Uart2_Rx() == (uint8_t)(Data_add & 0xFF))
            {
                if(Uart2_Rx() == (uint8_t)(Data_add >>8))
                {
                    if (Uart2_Rx() == Uart_Sync_Head2)
                    {
                        if (Uart2_Rx() == Uart_Sync_Head1)
                        {
                            s = RecData_Deal();

                            if (s != ERR_End)
                            {
                                UART2_SendData(Uart_Sync_Head1);
                                UART2_SendData(Uart_Sync_Head2);
                                UART2_SendData(0x00);
                                if (s == ERR_ERROR)
                                {
                                    UART2_SendData(0x01);
                                }
                                else
                                {
                                    UART2_SendData(0x00);
                                }
                                UART2_SendData(Uart_Sync_Head2);
                                UART2_SendData(Uart_Sync_Head1);
                            }
                        }
                    }
                }
            }
        }
    }
}



