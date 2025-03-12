/********************************** (C) COPYRIGHT  *******************************
 * File Name          : PD_Prot.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/04/06
 * Description        :
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#ifndef __PD_PROT_H__
#define __PD_PROT_H__

/*******************************************************************************/

#include "debug.h"
#include "libCH32_USBPD.h"
#include"ch32x035_usbpd.h"

/* Connection status change */
void pDevice_Attached(void);		//Equipment connection
void pDevice_Unattached(void);		//Device removal

/* Key message */
void pProt_IDLE(void);				//Message reception without state machine

/* Source power negotiation process */
void pProt_TX_SrcCap(void);
void pProt_Wait_Request(void);
void pProt_RX_Request(void);
void pProt_TX_Accept(void);
void pProt_Set_Volt_Change(void);
void pProt_Volt_Change(void);
void pProt_TX_PS_RDY(void);

/* Sink power negotiation process */
void pProt_Wait_SrcCap(void);
void pProt_RX_SrcCap(void);
void pProt_TX_Request(void);
void pProt_RX_Accept(void);
void pProt_RX_PS_RDY(void);

void pProt_TX_SinkCap(void);
void pProt_RX_DRSwap(void);
void pProt_TX_SoftRst(void);
void pProt_SoftRst_RX_Accept(void);
void pProt_RX_SoftRst(void);
void pProt_Excute_SoftRst(void);
void pProt_RX_ChunkedMsg(void);

typedef struct {
	union {
		u32 Data;
		struct {
			u32 MaxCurrent:10;
			u32 Voltage:10;
			u32 PeakCurrent:2;
			u32 Reserved:1;
			u32 EPRMode:1;
			u32 UnchunkedExtended:1;
			u32 DualRoleData:1;
			u32 USBComm:1;
			u32 UnconstrainedPower:1;
			u32 USBSuspend:1;
			u32 DualRolePower:1;
			u32 FixedSupply:2;
		};
	};
} st_SrcCap_Fixed;


#endif
