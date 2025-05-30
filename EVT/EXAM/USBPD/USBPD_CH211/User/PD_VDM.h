/********************************** (C) COPYRIGHT  *******************************
 * File Name          : PD_VDM.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/04/06
 * Description        :
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#ifndef __PD_VDM_H__
#define __PD_VDM_H__

/*******************************************************************************/

#include "debug.h"
#include"ch32x035_usbpd.h"

typedef struct {
	union {
		u16 Data[2];
		struct {
			u16 Command:5;
			u16 Reserved:1;
			u16 CommandType:2;
			u16 ObjectPosition:3;
			u16 VersionMinor:2;
			u16 VersionMajor:2;
			u16 Type:1;
			u16 SVID:16;
		};
	};
} st_VDM_Header;

typedef struct
{
	union {
		u8 Sta;
		struct {
			u8 Enter_Mode_already:1;
			u8 Explicit_Contract_Established:1;
			u8 empty:6;
		       };
	      };

} st_VDM;

void pProt_RX_VDM(void);
void pProt_RX_REQ_IDENT_UFP();
void pProt_RX_REQ_SVID_UFP();
void pProt_RX_REQ_MODE_UFP();
void pProt_RX_REQ_ENTER_UFP();
void pProt_RX_REQ_EXIT_UFP();
void pProt_RX_ATTENTION_UFP();
void pProt_RX_REQ_IDENT_DFP();
void pProt_RX_REQ_SVID_DFP();
void pProt_RX_REQ_MODE_DFP();
void pProt_RX_ATTENTION_DFP();
void pProt_TX_NAK(void);
void pProt_TX_ACK(void);
void pProt_TX_DISC_IDENT();
void pProt_RX_ACK_IDENT_DFP();
void pProt_RX_ACK_SVID_DFP();
void pProt_RX_ACK_MODE_DFP();
void pProt_RX_ACK_ENTER_DFP();
void pProt_RX_ACK_EXIT_DFP();
void pProt_RX_ACK_IDENT_UFP();
void pProt_RX_ACK_SVID_UFP();
void pProt_RX_ACK_MODE_UFP();


#endif
