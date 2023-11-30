/********************************** (C) COPYRIGHT  *******************************
 * File Name          : iap.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/04/06
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

#define FLASH_Base   0x08005000
#define USBD_DATA_SIZE   64
iapfun jump2app;
u32 Program_addr = FLASH_Base;
u32 Verity_addr = FLASH_Base;
u32 User_APP_Addr_offset = 0x5000;
u8 Verity_Star_flag = 0;
u8 Fast_Program_Buf[390];
u32 CodeLen = 0;
u8 End_Flag = 0;
u8 EP2_Rx_Buffer[USBD_DATA_SIZE];
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
 * @fn      RecData_Deal
 *
 * @brief   UART-USB deal date
 *
 * @return  ERR_ERROR - ERROR
 *          ERR_SCUESS - SCUESS
 *          ERR_End - End
 */

u8 RecData_Deal(void)
{
     u8 i, s, Lenth;

     Lenth = isp_cmd_t->Len;

     switch ( isp_cmd_t->Cmd) {
     case CMD_IAP_ERASE:
         FLASH_Unlock_Fast();
         s = ERR_SCUESS;
         break;

     case CMD_IAP_PROM:
         for (i = 0; i < Lenth; i++) {
             Fast_Program_Buf[CodeLen + i] = isp_cmd_t->data[i];
         }
         CodeLen += Lenth;
         if (CodeLen >= 256) {
             FLASH_Unlock_Fast();
             FLASH_ErasePage_Fast(Program_addr);
             CH32_IAP_Program(Program_addr, (u32*) Fast_Program_Buf);
             CodeLen -= 256;
             for (i = 0; i < CodeLen; i++) {
                 Fast_Program_Buf[i] = Fast_Program_Buf[256 + i];
             }

             Program_addr += 0x100;

         }
         s = ERR_SCUESS;
         break;

     case CMD_IAP_VERIFY:
         if (Verity_Star_flag == 0) {
             Verity_Star_flag = 1;

             for (i = 0; i < (256 - CodeLen); i++) {
                 Fast_Program_Buf[CodeLen + i] = 0xff;
             }

             FLASH_ErasePage_Fast(Program_addr);
             CH32_IAP_Program(Program_addr, (u32*) Fast_Program_Buf);
             CodeLen = 0;
         }

         s = ERR_SCUESS;
         for (i = 0; i < Lenth; i++) {
             if (isp_cmd_t->data[i] != *(u8*) (Verity_addr + i)) {
                 s = ERR_ERROR;
                 break;
             }
         }

         Verity_addr += Lenth;

         break;

     case CMD_IAP_END:
         Verity_Star_flag = 0;
         End_Flag = 1;
         Program_addr = FLASH_Base;
         Verity_addr = FLASH_Base;
         FLASH->CTLR |= ((uint32_t)0x00008000);  //FLASH_Lock_Fast
         FLASH->CTLR |= ((uint32_t)0x00000080);  //FLASH_Lock
         s = ERR_End;
         break;

     default:
         s = ERR_ERROR;
         break;
     }

     return s;
}

/*********************************************************************
 * @fn      GPIO_Cfg_init
 *
 * @brief   GPIO init
 *
 * @return  none
 */
void GPIO_Cfg_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

}

/*********************************************************************
 * @fn      GPIO_Cfg_Float
 *
 * @brief   GPIO float
 *
 * @return  none
 */
void GPIO_Cfg_Float(void)
{
    GPIO_DeInit(GPIOA);
    GPIO_DeInit(GPIOC);
    GPIO_AFIODeInit();
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, DISABLE);
}

/*********************************************************************
 * @fn      PA0_Check
 *
 * @brief   Check PA0 state
 *
 * @return  1 - IAP
 *          0 - APP
 */
u8 PA0_Check(void)
{
    u8 i, cnt=0;

    GPIO_Cfg_init();

    for(i=0; i<10; i++){
        if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==0) cnt++;
        Delay_Ms(5);
    }

    if(cnt>6) return 0;
    else return 1;
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
 * @fn      UART2_SendMultiyData
 *
 * @brief   Deal device Endpoint 3 OUT.
 *
 * @param   l: Data length.
 *
 * @return  none
 */
void UART2_SendMultiyData(u8* pbuf, u8 num)
{
    u8 i = 0;

    while(i<num)
    {
        while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
        USART_SendData(USART2, pbuf[i]);
        i++;
    }
}
/*********************************************************************
 * @fn      UART2_SendMultiyData
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
    u8 Data_add = 0;

    if (Uart2_Rx() == Uart_Sync_Head1)
    {
        if (Uart2_Rx() == Uart_Sync_Head2)
        {
            isp_cmd_t->Cmd = Uart2_Rx();
            Data_add += isp_cmd_t->Cmd;
            isp_cmd_t->Len = Uart2_Rx();
            Data_add += isp_cmd_t->Len;
            isp_cmd_t->Rev[0] = Uart2_Rx();
            Data_add += isp_cmd_t->Rev[0];
            isp_cmd_t->Rev[1] = Uart2_Rx();
            Data_add += isp_cmd_t->Rev[1];

            if ((isp_cmd_t->Cmd == CMD_IAP_PROM) || (isp_cmd_t->Cmd == CMD_IAP_VERIFY))
            {
                for (i = 0; i < isp_cmd_t->Len; i++) {
                    isp_cmd_t->data[i] = Uart2_Rx();
                    Data_add += isp_cmd_t->data[i];
                }
            }

            if (Uart2_Rx() == Data_add)
            {
                s = RecData_Deal();

                if (s != ERR_End)
                {
                    UART2_SendData(0x00);
                    if (s == ERR_ERROR)
                    {
                        UART2_SendData(0x01);
                    }
                    else
                    {
                        UART2_SendData(0x00);
                    }
                }
            }
        }
    }
}

void SW_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      SW_Handler
 *
 * @brief   This function handles Software exception.
 *
 * @return  none
 */
void SW_Handler(void) {
    __asm("li  a6, 0x5000");
    __asm("jr  a6");

    while(1);
}


