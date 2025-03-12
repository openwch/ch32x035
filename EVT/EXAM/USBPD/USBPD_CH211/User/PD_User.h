/********************************** (C) COPYRIGHT *******************************
* File Name          : PD_User.H
* Author             : WCH
* Version            : V1.0
* Date               : 2022/01/13
* Description        : 
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
********************************************************************************/

#ifndef __PD_USER_H__
#define __PD_USER_H__

/*******************************************************************************/

#include "debug.h"
#include "ch32x035_usbpd.h"
#include "CH211_I2C.h"
#include "libCH32_USBPD.h"
#include "PD_Prot.H"

//#define 	DEBUG_Print(format, ...) 		printf(format, ##__VA_ARGS__)
#define 	DEBUG_Print(format, ...)
#define 	DEF_EN_VOLTCHANGE	0
#define 	DEF_EN_VCONN		0
//#define 	DEF_EN_BIST			1

#define LED_R           GPIO_Pin_4
#define LED_G           GPIO_Pin_3
#define LED_B           GPIO_Pin_2

void PD_User_Snk_DevIn(void);
void PD_User_Snk_Rx_SrcCap(void);
void PD_User_Snk_Rx_PS_RDY(void);
void PD_User_Src_DevIn(void);
void PD_User_Src_VoltChange(void);
void PD_User_DevOut(void);
void PD_User_Timer(void);

#endif
