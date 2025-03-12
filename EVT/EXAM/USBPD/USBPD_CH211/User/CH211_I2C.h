/********************************** (C) COPYRIGHT *******************************
* File Name          : CH211_I2C.H
* Author             : WCH
* Version            : V1.0
* Date               : 2022/01/13
* Description        : 
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
********************************************************************************/
#ifndef __CH211_I2C_H__
#define __CH211_I2C_H__

#include "debug.h"

#define     SCL_SET_IPU     {GPIOA->CFGLR &= ~(GPIO_CFGLR_CNF0|GPIO_CFGLR_MODE0) ; GPIOA->CFGLR |= GPIO_CFGLR_CNF0_1 ; GPIOA->OUTDR |= GPIO_OUTDR_ODR0;}
#define     SCL_SET_PP      {GPIOA->CFGLR &= ~(GPIO_CFGLR_CNF0|GPIO_CFGLR_MODE0) ; GPIOA->CFGLR |= GPIO_CFGLR_MODE0_0 ; GPIOA->OUTDR |= GPIO_OUTDR_ODR0;}
#define     SCL_PP_H        GPIOA->OUTDR |= GPIO_OUTDR_ODR0
#define     SCL_PP_L        GPIOA->OUTDR &= ~GPIO_OUTDR_ODR0

#define     SDA_SET_IPU     {GPIOA->CFGLR &= ~(GPIO_CFGLR_CNF1|GPIO_CFGLR_MODE1) ; GPIOA->CFGLR |= GPIO_CFGLR_CNF1_1 ; GPIOA->OUTDR |= GPIO_OUTDR_ODR1;}
#define     SDA_SET_PP      {GPIOA->CFGLR &= ~(GPIO_CFGLR_CNF1|GPIO_CFGLR_MODE1) ; GPIOA->CFGLR |= GPIO_CFGLR_MODE1_0 ; GPIOA->OUTDR |= GPIO_OUTDR_ODR1;}
#define     SDA_PP_H        GPIOA->OUTDR |= GPIO_OUTDR_ODR1
#define     SDA_PP_L        GPIOA->OUTDR &= ~GPIO_OUTDR_ODR1

#define     SDA_IN          (GPIOA->INDR&GPIO_INDR_IDR1)?(1):(0)

#define		I2C_Delay       {asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}
//#define     I2C_Delay       {asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}
//#define     I2C_Delay       Delay_Us(1)

#define     PIN_STAT         0
#define     CCI2            0x80
#define     CCI1            0x40
#define     VBUS_OV         0x20
#define     VBUS_RDY        0x10
#define     HVHI            0x08
#define     HVLI            0x04
#define     KEYHI           0x02
#define     KEYLI           0x01

#define     PIN_CFG          1
#define     CC2_IE          0x80
#define     CC1_IE          0x40
#define     HVIO_IE         0x20
#define     KEY_IE          0x10
#define     VBUS_DOWN_IE    0x08
#define     SDA_PU          0x04
#define     INT_PIN         0x03
#define     INT_PIN_HVIO    0x02
#define     INT_PIN_SCL     0x01

#define     CC_CTRL          2
#define     CC2_VCE         0x80
#define     CC2_PD          0x40
#define     CC2_OE          0x20
#define     CC2_GE          0x10
#define     CC1_VCE         0x08
#define     CC1_PD          0x04
#define     CC1_OE          0x02
#define     CC1_GE          0x01

#define     OD_CTRL          3
#define     OD2_OE          0x80
#define     OD2_GE          0x40
#define     OD2_PE          0x20
#define     OD1_OE          0x08
#define     OD1_GE          0x04
#define     OD1_PE          0x02

#define     HVCP_CTRL        4
#define     VBUS_DISC       0x80
#define     CP_AUTO         0x10
#define     CP_AE           0x08
#define     CP_PU           0x04
#define     CP_LE           0x02
#define     CP_LX           0x01

#define     HVIO_KEY         5
#define     KEY_PD          0x20
#define     KEY_OE          0x10
#define     HV_PU           0x04
#define     HV_PD           0x02
#define     HV_OE           0x01

#define     SYS_CFG          6
#define     LDO33_OFF       0x80
#define     CC_HVT3V        0x40
#define     CPLE_OVOT       0x20
#define     RST_OV          0x10
#define     LDO33_WAKE      0x08
#define     LDO_VSEL        0x07

#define     LDO_VSEL_3V3    0x00
#define     LDO_VSEL_3V0    0x01
#define     LDO_VSEL_2V7    0x02
#define     LDO_VSEL_2V4    0x03
#define     LDO_VSEL_3V6    0x04
#define     LDO_VSEL_3V9    0x05
#define     LDO_VSEL_4V2    0x06
#define     LDO_VSEL_4V5    0x07

#define     SYS_STAT         7
#define     LDO33_OFF       0x80
#define     OT_RST          0x40
#define     VBUS_OV         0x20
#define     VBUS_RDY        0x10
#define     VBUS_LAST       0x08
#define     VSYS_RDY        0x01

#define PD_Enable_VBUS      GPIO_SetBits(GPIOB, GPIO_Pin_3)
#define PD_Disable_VBUS     GPIO_ResetBits(GPIOB, GPIO_Pin_3)

#define I2C_SCL_H     		{GPIOA->CFGLR&=~0xF0000;GPIOA->CFGLR|=0x40000;}
#define I2C_SCL_L       	{GPIOA->CFGLR&=~0xF0000;GPIOA->CFGLR|=0x10000;}

#define I2C_SDA_H       	{GPIOA->CFGLR&=~0xF00000;GPIOA->CFGLR|=0x400000;}
#define I2C_SDA_L       	{GPIOA->CFGLR&=~0xF00000;GPIOA->CFGLR|=0x100000;}
#define I2C_SDA_IN      	GPIOA->INDR&0x20

#define I2C_SDA         	GPIO_Pin_1
#define I2C_SCL         	GPIO_Pin_0

#define I2C_Addr       		0x35

/*I2C*/
void I2C_PWR_Init (void);
void CH211_I2C_Init(void);
void CH211_I2C_WriteByte( u8 addr , u8 byte );
u8 CH211_I2C_ReadByte( u8 addr );
void CH211_I2C_SetBit( u8 addr , u8 setbits , u8 resetbits );
void CH211_Param_Init(void);

#endif
