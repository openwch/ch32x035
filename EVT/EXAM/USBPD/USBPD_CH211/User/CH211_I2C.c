/********************************** (C) COPYRIGHT *******************************
* File Name          : CH211_I2C.C
* Author             : WCH
* Version            : V1.0
* Date               : 2022/04/28
* Description        : 
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
********************************************************************************/

#include <stdio.h>
#include <string.h>
#include "debug.h"
#include "CH211_I2C.h"

/*********************************************************************
 * @fn      I2C_PWR_Init
 *
 * @brief   CH211 initialization
 *
 * @return  none
 */
void I2C_PWR_Init (void)
{
	GPIO_InitTypeDef GPIO_InitStructure = {0};

	GPIO_ResetBits(GPIOA, I2C_SCL|I2C_SDA);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = I2C_SCL|I2C_SDA;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	Delay_Ms(10);

	CH211_Param_Init();
}

/*********************************************************************
 * @fn      I2C_Start
 *
 * @brief   I2C start signal
 *
 * @return  none
 */
void I2C_Start(void)
{
    SCL_SET_PP;
    SDA_SET_PP;
    I2C_Delay;

    SDA_PP_L;
    I2C_Delay;
    SCL_PP_L;
}

/*********************************************************************
 * @fn      I2C_Stop
 *
 * @brief   I2C stop signal
 *
 * @return  none
 */
void I2C_Stop(void)
{
    SDA_PP_L;
    SCL_PP_H;
    I2C_Delay;

    SDA_PP_H;
    I2C_Delay;

    SCL_SET_IPU;
    SDA_SET_IPU;
}

/*********************************************************************
 * @fn      I2C_Send_Byte
 *
 * @brief   Send byte to register
 *
 * @return  byte
 */
void I2C_Send_Byte(u8 byte)
{
    for (u8 i=0;i<8;i++)
    {
    	( byte&0x80 )?( SDA_PP_H ):( SDA_PP_L );
        byte<<=1;
        I2C_Delay;

        SCL_PP_H;
        I2C_Delay;

        SCL_PP_L;
    }

    SDA_SET_IPU;
    I2C_Delay;

    SCL_PP_H;
    I2C_Delay;
    SCL_PP_L;
    I2C_Delay;

    SDA_SET_PP;
    SDA_PP_L;
}

/*********************************************************************
 * @fn      I2C_Read_Byte
 *
 * @brief   Read byte from register
 *
 * @return  byte
 */
u8 I2C_Read_Byte(void)
{
    u8 byte = 0;

    SDA_SET_IPU;
    for (u8 i=0;i<8;i++)
    {
    	for (u8 j=0;j<5;j++) I2C_Delay;
        byte<<=1;
        byte += SDA_IN;

        SCL_PP_H;
        I2C_Delay;

        SCL_PP_L;
    }

    SDA_SET_PP;
    SDA_PP_L;

    I2C_Delay;

    SCL_PP_H;
    I2C_Delay;
    SCL_PP_L;
    I2C_Delay;

    return byte;
}

/*********************************************************************
 * @fn      CH211_I2C_WriteByte
 *
 * @brief   Write value to register
 *
 * @param   addr - register addr
 * 			byte - value
 *
 * @return  none
 */
void CH211_I2C_WriteByte( u8 addr , u8 byte )
{
    I2C_Start();
    I2C_Send_Byte( I2C_Addr<<1 );
    I2C_Send_Byte( addr );
    I2C_Send_Byte( byte );
    I2C_Stop();
}

/*********************************************************************
 * @fn      CH211_I2C_ReadByte
 *
 * @brief   Read value from register
 *
 * @param   addr - register addr
 *
 * @return  register value
 */
u8 CH211_I2C_ReadByte( u8 addr )
{
    u8 byte;
    I2C_Start();
    I2C_Send_Byte( I2C_Addr<<1 );
    I2C_Send_Byte( addr );
    I2C_Stop();
    I2C_Start();
    I2C_Send_Byte( (I2C_Addr<<1) + 1 );
    byte = I2C_Read_Byte();
    I2C_Stop();

    return byte;
}

/*********************************************************************
 * @fn      CH211_I2C_SetBit
 *
 * @brief   Set the CH211 register bit by bit, read before write, and do not change the bits that are not operated on.
 * 			The operation of multi-bit registers should use both reset and set to achieve the correct setting effect, for example:
 * 		 	CH211_I2C_SetBit( PIN_CFG , INT_PIN_HVIO , INT_PIN )
 * 			CH211_I2C_SetBit( SYS_CFG , LDO_VSEL_3V6 , LDO_VSEL )
 *
 * @param   addr      - register addr.
 * 			setbits   - set bits.
 * 			resetbits - reset bits.
 * @return  none
 */
void CH211_I2C_SetBit( u8 addr , u8 setbits , u8 resetbits )
{
    u8 byte,newbyte;

    byte = CH211_I2C_ReadByte( addr );

    newbyte = (byte&(~resetbits))|setbits;
    if ( newbyte == byte ) return;

    CH211_I2C_WriteByte( addr , newbyte );
}

/*********************************************************************
 * @fn      CH211_Param_Init
 *
 * @brief   HVCP strong pull-down, drive low level.
 * 			CCL and CCH connections of CH211.
 *
 * @return  none
 */
void CH211_Param_Init(void)
{
	CH211_I2C_WriteByte(HVCP_CTRL, CP_LE | CP_LX);
	CH211_I2C_WriteByte(CC_CTRL, CC2_GE|CC1_GE|CC2_OE|CC1_OE);
}
