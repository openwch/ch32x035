/********************************** (C) COPYRIGHT *******************************
* File Name          : PD_User.C
* Author             : WCH
* Version            : V1.0.1
* Date               : 2025/10/27
* Description        : 
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
********************************************************************************/

#include <stdio.h>
#include <string.h>
#include "debug.h"
#include "PD_User.H"

//The device acts as a source, sending the Source_Capabilities message content
u32 SrcCap[] = 	{0x3A21912C,};	//5V3A
vu8 SrcCapCnt = 1;

//The device acts as a sink, sending the content of the Sink_Capabilities message.
u32 SinkCap[] = {0x2A01912C,};	//5V3A
vu8 SinkCapCnt = 1;

/*********************************************************************
 * @fn      PD_User_Snk_DevIn
 *
 * @brief   The device is connected as a sink.
 *
 * @return  none
 */
void PD_User_Snk_DevIn(void)
{
	GPIO_SetBits(GPIOA, LED_B );		//Blue LED
	GPIO_ResetBits(GPIOA, LED_G );		//Green LED
}

/*********************************************************************
 * @fn      PD_User_Snk_Rx_SrcCap
 *
 * @brief   The device receives the Source_Capabilities message as a sink.
 *
 * @return  none
 */
void PD_User_Snk_Rx_SrcCap(void)
{

}

/*********************************************************************
 * @fn      PD_User_Snk_Rx_PS_RDY
 *
 * @brief   The device receives the PS_RDY message as a sink.
 *
 * @return  none
 */
void PD_User_Snk_Rx_PS_RDY(void)
{

}

/*********************************************************************
 * @fn      PD_User_Src_DevIn
 *
 * @brief   The device is connected as a source.
 *
 * @return  none
 */
void PD_User_Src_DevIn(void)
{
	GPIO_SetBits(GPIOA, LED_B );		//Blue LED
	GPIO_ResetBits(GPIOA, LED_R );		//Red LED
}

/*********************************************************************
 * @fn      PD_User_Src_VoltChange
 *
 * @brief   The device is used as a source for voltage regulation.
 *
 * @return  none
 */

void PD_User_Src_VoltChange(void)
{
#if	( !DEF_EN_VOLTCHANGE )
	pProt_TX_PS_RDY();		//No pressure regulation
#else

	st_SrcCap_Fixed *tSrcCap = (st_SrcCap_Fixed *)&SrcCap[PD_PHY.savedRequest.ObjectPos-1];


	if ( PD_PHY.tSrcTransition ) {
		PD_PHY.tSrcTransition--;
		if ( !PD_PHY.tSrcTransition ) {

		}
	}
	else if ( PD_PHY.tPSTransition ) {

		float dat;
		PD_PHY.tPSTransition--;
		dat=GetADC_VBUS/4096*3.3/18*118;
		dat=dat*1000;
		if ( ( (dat > (tSrcCap->Voltage*50*0.95) ) && (dat < (tSrcCap->Voltage*50*1.05) ) ) || !PD_PHY.tPSTransition ) {
			pProt_TX_PS_RDY();
		}

	}
#endif
}

/*********************************************************************
 * @fn      PD_User_DevOut
 *
 * @brief   Device detection removal.
 *
 * @return  none
 */
void PD_User_DevOut(void)
{
	GPIO_SetBits(GPIOA, LED_G );	//Green LED OFF
	GPIO_SetBits(GPIOA, LED_R );	//Red LED OFF
}

/*********************************************************************
 * @fn      PD_User_DevOut
 *
 * @brief   User function, execute once every 1ms.
 *
 * @return  none
 */
void PD_User_Timer(void)
{

}
