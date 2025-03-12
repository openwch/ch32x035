/********************************** (C) COPYRIGHT  *******************************
 * File Name          : PD_Prot.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/04/06
 * Description        :
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#include <stdio.h>
#include <string.h>
#include "debug.h"
#include "PD_Prot.h"
#include "PD_User.H"
#include "PD_VDM.h"

extern st_VDM VDM_State;

enum enum_PD_Ctrl {
	enum_PD_Ctrl_Ignore,
	enum_PD_Ctrl_Reserved,
	enum_PD_Ctrl_txSoftReset,

	enum_PD_Ctrl_GetSrcCap,
	enum_PD_Ctrl_GetSinkCap,
	enum_PD_Ctrl_DRSwap,
	enum_PD_Ctrl_PRSwap,
	enum_PD_Ctrl_SoftReset,
	enum_PD_Ctrl_GetSinkCapExt,
	enum_PD_Ctrl_GetSrcInfo,
	enum_PD_Ctrl_GetRevision,
};

void (*Idle_CtrlMsg_Handle[][2])() = {		//Must correspond to the enum
									/* Source */			 /* Sink */
/* enum_PD_Ctrl_Ignore	    	*/	Prot_NULL,				Prot_NULL,
/* enum_PD_Ctrl_Reserved		*/	PD_RX_Reserved,			PD_RX_Reserved,
/* enum_PD_Ctrl_txSoftReset		*/	pProt_TX_SoftRst,		pProt_TX_SoftRst,

/* enum_PD_Ctrl_GetSrcCap		*/	pProt_TX_SrcCap,		pProt_TX_SrcCap,
/* enum_PD_Ctrl_GetSinkCap		*/	pProt_TX_SinkCap,		pProt_TX_SinkCap,
/* enum_PD_Ctrl_DRSwap	    	*/	pProt_RX_DRSwap,		pProt_RX_DRSwap,
/* enum_PD_Ctrl_PRSwap	    	*/	PD_TX_Reject,			PD_TX_Reject,
/* enum_PD_Ctrl_SoftReset		*/	pProt_RX_SoftRst,		pProt_RX_SoftRst,
/* enum_PD_Ctrl_GetSinkCapExt	*/	PD_TX_SinkCapExt,		PD_TX_SinkCapExt,
/* enum_PD_Ctrl_GetSrcInfo		*/	PD_TX_SrcInfo,			PD_RX_Reserved,
/* enum_PD_Ctrl_GetRevision		*/	PD_TX_Revision,			PD_TX_Revision,
};

const u8 enum_Idle_CtrlMsg[] = {
/* PD_Ctrl_Reserved			 	*/		enum_PD_Ctrl_Reserved,
/* PD_Ctrl_GoodCRC			 	*/		enum_PD_Ctrl_Ignore,
/* PD_Ctrl_GotoMin			 	*/		enum_PD_Ctrl_Ignore,
/* PD_Ctrl_Accept			 	*/		enum_PD_Ctrl_txSoftReset,
/* PD_Ctrl_Reject			 	*/		enum_PD_Ctrl_Ignore,
/* PD_Ctrl_Ping				 	*/		enum_PD_Ctrl_Ignore,
/* PD_Ctrl_PS_Ready			 	*/		enum_PD_Ctrl_Ignore,
/* PD_Ctrl_GetSrcCap		 	*/		enum_PD_Ctrl_GetSrcCap,
/* PD_Ctrl_GetSinkCap		 	*/		enum_PD_Ctrl_GetSinkCap,
/* PD_Ctrl_DRSwap			 	*/		enum_PD_Ctrl_DRSwap,
/* PD_Ctrl_PRSwap			 	*/		enum_PD_Ctrl_PRSwap,
/* PD_Ctrl_VconnSwap		 	*/		enum_PD_Ctrl_Reserved,			//Choose Reject or NotSupported based on the protocol version
/* PD_Ctrl_Wait				 	*/		enum_PD_Ctrl_Ignore,
/* PD_Ctrl_SoftReset		 	*/		enum_PD_Ctrl_SoftReset,
/* PD_Ctrl_DataReset		 	*/		enum_PD_Ctrl_Reserved,
/* PD_Ctrl_DataResetComplete 	*/		enum_PD_Ctrl_Ignore,
/* PD_Ctrl_NotSupported			*/		enum_PD_Ctrl_Ignore,
/* PD_Ctrl_GetSrcCapExtended	*/		enum_PD_Ctrl_Reserved,
/* PD_Ctrl_GetStatus			*/		enum_PD_Ctrl_Reserved,
/* PD_Ctrl_FRSwap				*/		enum_PD_Ctrl_Reserved,
/* PD_Ctrl_GetPPSStatus			*/		enum_PD_Ctrl_Ignore,
/* PD_Ctrl_GetCountryCodes		*/		enum_PD_Ctrl_Reserved,
/* PD_Ctrl_GetSinkCapExtended	*/		enum_PD_Ctrl_GetSinkCapExt,
/* PD_Ctrl_GetSrcInfo			*/		enum_PD_Ctrl_GetSrcInfo,
/* PD_Ctrl_GetRevision			*/		enum_PD_Ctrl_GetRevision,
};

enum enum_PD_Data {
	enum_PD_Data_Ignore,
	enum_PD_Data_Reserved,
	enum_PD_Data_txSoftReset,

	enum_PD_Data_SrcCap,
	enum_PD_Data_Request,
	enum_PD_Data_BIST,
	enum_PD_Data_VendorDefined,
};

void (*Idle_DataMsg_Handle[][2])() = {		//Must correspond to the enum
									/* Source */			 /* Sink */
/* enum_PD_Data_Ignore	    	*/	Prot_NULL,				Prot_NULL,
/* enum_PD_Data_Reserved		*/	PD_RX_Reserved,			PD_RX_Reserved,
/* enum_PD_Data_txSoftReset		*/	pProt_TX_SoftRst,		pProt_TX_SoftRst,

/*enum_PD_Data_SrcCap	    	*/	Prot_NULL,				pProt_RX_SrcCap,
/*enum_PD_Data_Request	    	*/	pProt_RX_Request,		Prot_NULL,
/*enum_PD_Data_BIST	        	*/	PD_TX_BIST,				PD_TX_BIST,
/*enum_PD_Data_VendorDefined	*/	pProt_RX_VDM,			pProt_RX_VDM,
};

const u8 enum_Idle_DataMsg[] = {
/* PD_Data_Reserved1		*/		enum_PD_Data_Reserved,
/* PD_Data_SrcCap			*/		enum_PD_Data_SrcCap,
/* PD_Data_Request			*/		enum_PD_Data_Request,
/* PD_Data_BIST				*/		enum_PD_Data_BIST,
/* PD_Data_SinkCap			*/		enum_PD_Data_Ignore,
/* PD_Data_BatteryStatus	*/		enum_PD_Data_Ignore,
/* PD_Data_Alert			*/		enum_PD_Data_Ignore,
/* PD_Data_GetCountryInfo	*/		enum_PD_Data_Ignore,
/* PD_Data_EnterUSB			*/		enum_PD_Data_Ignore,
/* PD_Data_EPRRequest		*/		enum_PD_Data_Ignore,
/* PD_Data_EPRMode			*/		enum_PD_Data_Ignore,
/* PD_Data_SrcInfo			*/		enum_PD_Data_Ignore,
/* PD_Data_Revision			*/		enum_PD_Data_Ignore,
/* PD_Data_Reserved2		*/		enum_PD_Data_Reserved,
/* PD_Data_Reserved3		*/		enum_PD_Data_Reserved,
/* PD_Data_VendorDefined	*/		enum_PD_Data_VendorDefined,
};

/*********************************************************************
 * @fn      pDevice_Attached
 *
 * @brief   Check device connection.
 *
 * @return  none
 */
void pDevice_Attached(void)
{
	PD_PHY_Reset_Value( PD_DEVICE.DevStat );	//Reset PD PHY related counters
	NVIC_EnableIRQ( USBPD_IRQn );				//Enable PD interrupt

	PD_Disable_Discharge;
	PD_DEVICE.ConnectStat = 1;
	VDM_State.Explicit_Contract_Established = 0;
	VDM_State.Enter_Mode_already = 0;

    if ( PD_DEVICE.DevStat ) {			//Src
    	PD_User_Src_DevIn();
    	( DEF_EN_VCONN )?( PD_TX_VCONN_DISC_IDENT() ):( pProt_TX_SrcCap() );

    }
    else {								//Sink
    	PD_User_Snk_DevIn();
    	pProt_Wait_SrcCap();
    }
}

/*********************************************************************
 * @fn      pDevice_Unattached
 *
 * @brief   Check device removal.
 *
 * @return  none
 */
void pDevice_Unattached(void)
{
	NVIC_DisableIRQ( USBPD_IRQn );			//Turn off PD interrupt

	PD_PHY.WaitTxGcrc = PD_PHY.WaitRxGcrc = PD_PHY.WaitMsgTx = PD_PHY.WaitMsgRx = 0;
	PD_DEVICE.ConnectStat = 0;
	VDM_State.Explicit_Contract_Established = 0;
	VDM_State.Enter_Mode_already = 0;

	PD_User_DevOut();
}

/*********************************************************************
 * @fn      pProt_IDLE
 *
 * @brief   Message reception without state machine.
 *
 * @return  none
 */
void pProt_IDLE(void)
{

	if ( rxHeader->Extended ) {
		if ( rxExtHeader->Chunked ) DEBUG_Print("rxDataSize.%d\r\nrxChunkNumber.%d\r\n",rxExtHeader->DataSize,rxExtHeader->ChunkNumber);
		if ( rxExtHeader->Chunked && ( rxExtHeader->DataSize > ( rxExtHeader->ChunkNumber*26 + rxHeader->NDO*4 - 2 ) ) ) PD_TX_ChunkRequest();
		else if ( rxExtHeader->ChunkNumber == 0 ) {
			if ( rxHeader->MsgType == PD_Ext_GetBatteryCap )		PD_TX_NotSupported();
			if ( rxHeader->MsgType == PD_Ext_GetBatteryStatus )		PD_TX_NotSupported();
			if ( rxHeader->MsgType == PD_Ext_SecurityRequest )		PD_TX_NotSupported();
			if ( rxHeader->MsgType == PD_Ext_GetManufacturerInfo )	PD_TX_ManufacturerInfo();
		}
		else PD_RX_Reserved();
	}
	//Data Message
	else if ( rxHeader->NDO ) {
		if ( Check_DataMsg_Type_Reserved ) PD_RX_Reserved();		//Protocol limit of PD2.0 or 3.0
		else ( Idle_DataMsg_Handle[ enum_Idle_DataMsg[rxHeader->MsgType] ][ (PD_PHY.Header.PortPwrRole)?(0):(1) ] )();
	}
	//Control Message
	else {
		if ( Check_CtrlMsg_Type_Reserved ) PD_RX_Reserved();		//Protocol limit of PD2.0 or 3.0
		else ( Idle_CtrlMsg_Handle[ enum_Idle_CtrlMsg[rxHeader->MsgType] ][ (PD_PHY.Header.PortPwrRole)?(0):(1) ] )();
	}
}

/*********************************************************************
 * @fn      pProt_RX_DRSwap
 *
 * @brief   Receive DRSwap,Switch data roles.
 *
 * @return  none
 */
void pProt_RX_DRSwap(void)
{
  if( !(VDM_State.Enter_Mode_already) )
  {
	  if(PD_PHY.Header.PortPwrRole)
	  {
		PD_PHY_Header_Init(1,0,(PD_PHY.Header.PortDataRole)?(PD_Ctrl_Accept):(PD_Ctrl_Reject));
		PD_Prot_pSet( NULL , pProt_IDLE , pProt_TX_SoftRst , NULL );
		PD_PHY.Header.PortDataRole = 1-PD_PHY.Header.PortDataRole;
	  }
	  else
	  {
		  PD_TX_Reject();
	  }
  }
  else
  {
	  PD_TX_HRST();
  }
}

/*********************************************************************
 * @fn      pProt_TX_SinkCap
 *
 * @brief   Send SinkCap.
 *
 * @return  none
 */
void pProt_TX_SinkCap(void)
{
	PD_PHY_Header_Init(1,SinkCapCnt,PD_Data_SinkCap);		//Send SinkCap 1ms later
	memcpy(&PD_TX_BUF[1],SinkCap,SinkCapCnt*4);
	//	Transmission successful pProt_TX_PS_RDY		Received successfully; analyze whether it is a request		Send failed SoftRST		Receive timeout SoftRST
	PD_Prot_pSet( NULL , pProt_IDLE , pProt_TX_SoftRst , NULL );
}

/*********************************************************************
 * @fn      pProt_TX_SrcCap
 *
 * @brief   Send SrcCap.
 *
 * @return  none
 */
void pProt_TX_SrcCap(void)
{
	PD_PHY_Header_Init(1,SrcCapCnt,PD_Data_SrcCap);		//Send SrcCap after 1ms
	memcpy(&PD_TX_BUF[1],SrcCap,SrcCapCnt*4);
	if ( PD_PHY.SrcCapCnt ) {	//SrcCap at startup: 120ms interval, no reset on failure to send
		PD_PHY.SrcCapCnt --;
		PD_PHY.WaitMsgRx = 0;
		PD_PHY.MsgTxCnt = 120;
		//PD_PHY.TxMsgID ++;
		PD_Prot_pSet( pProt_Wait_Request , (PD_PHY.SrcCapCnt)?(pProt_RX_Request):(pProt_IDLE) , (PD_PHY.SrcCapCnt)?(pProt_TX_SrcCap):(NULL) , NULL );
	}
	else if ( PD_PHY.Header.PortPwrRole ) pProt_Wait_Request();			//SrcCap in Source protocol: Send immediately, send failure SoftRST
	else PD_Prot_pSet( NULL , pProt_IDLE , pProt_TX_SoftRst , pProt_TX_SoftRst );
	//	Send success don't care   Receive success jump to pProt_RX_Request     Send failure SoftRST      Receive timeout SoftRST
}

/*********************************************************************
 * @fn      pProt_Wait_Request
 *
 * @brief   Wait Request.
 *
 * @return  none
 */
void pProt_Wait_Request(void)
{
	//Receive Request within 30ms
	PD_PHY.WaitMsgRx = 1;
	PD_PHY.MsgRxCnt = 28;
	PD_Prot_pSet( NULL , pProt_RX_Request , pProt_TX_SoftRst , PD_TX_HRST );
	//	Send success don't care   Receive success jump to pProt_RX_Request   Send failure SoftRST  Receive timeout SoftRST
}

/*********************************************************************
 * @fn      pProt_RX_Request
 *
 * @brief   Receive Request.
 *
 * @return  none
 */
void pProt_RX_Request(void)
{
	if ( rxHeader->MsgType == PD_Data_Request ) {
		st_Request_Fixed tRequest;
		PD_PHY.SrcCapCnt = 0;
		PD_PHY.WaitMsgRx = 0;
		tRequest.Data = (PD_RX_BUF[2]<<16) + PD_RX_BUF[1];
		if ( !tRequest.ObjectPos || ( tRequest.ObjectPos > (sizeof(SrcCap)/4) ) ) PD_TX_Reject();		//First judge ObjectPos separately to prevent overflow when initializing the structure.
		else {
			st_SrcCap_Fixed *tSrcCap = (st_SrcCap_Fixed *)&SrcCap[tRequest.ObjectPos-1];		//Create the parser structure
			if ( tRequest.Current > tSrcCap->MaxCurrent ) {
				DEBUG_Print("Req Reject:%d,%d,%d,%d\r\n",tRequest.ObjectPos,tRequest.Current,tRequest.MaxCurrent,tSrcCap->MaxCurrent);
				PD_TX_Reject();
			}
			else {
				PD_PHY.savedRequest.Data = tRequest.Data;
				pProt_TX_Accept();
			}
		}
	}
	else {
		PD_PHY.WaitMsgRx = 0;
		pProt_TX_SoftRst();
	}
}

/*********************************************************************
 * @fn      pProt_TX_Accept
 *
 * @brief   Send Accept.
 *
 * @return  none
 */
void pProt_TX_Accept(void)
{
	DEBUG_Print("TX Accept\r\n");
	PD_PHY_Header_Init(1,0,PD_Ctrl_Accept);		//Send Accept after 1ms

	//	Send success pProt_TX_PS_RDY; Receive success analyze whether it is a request; Send failure SoftRST; Receive timeout SoftR
	PD_Prot_pSet( pProt_Set_Volt_Change , pProt_RX_Request , pProt_TX_SoftRst , pProt_TX_SoftRst );

	//	Send success pProt_TX_PS_RDY; Receive success analyze whether it is a request; Send failure SoftRST; Receive timeout SoftRST
	//PD_Prot_pSet( pProt_TX_PS_RDY , pProt_RX_Request , pProt_TX_SoftRst , pProt_TX_SoftRst );
}

/*********************************************************************
 * @fn      pProt_Set_Volt_Change
 *
 * @brief   Set the conditions for voltage change.
 *
 * @return  none
 */
void pProt_Set_Volt_Change(void)
{
	PD_PHY.VoltChanging = 1;
	PD_PHY.tSrcTransition = 35;
	PD_PHY.tPSTransition = 470;
}

/*********************************************************************
 * @fn      pProt_Volt_Change
 *
 * @brief   Voltage change.
 *
 * @return  none
 */
void pProt_Volt_Change(void)
{
	PD_User_Src_VoltChange();
}

/*********************************************************************
 * @fn      pProt_TX_PS_RDY
 *
 * @brief   Send PS_RDY.
 *
 * @return  none
 */
void pProt_TX_PS_RDY(void)
{
	PD_PHY.VoltChanging = 0;
	VDM_State.Explicit_Contract_Established = 1;
	PD_PHY_Header_Init(35,0,PD_Ctrl_PS_Ready);		//Send Accept after 1ms
	//	Send success pProt_TX_PS_RDY; Receive success analyze whether it is a request; Send failure SoftRST; Receive timeout SoftRST
	PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );
}

/*********************************************************************
 * @fn      pProt_Wait_SrcCap
 *
 * @brief   Wait SrcCap.
 *
 * @return  none
 */
void pProt_Wait_SrcCap(void)
{
//Received Accept within 465ms
	PD_PHY.WaitMsgRx = 1;
	PD_PHY.MsgRxCnt = 465;
//	Send success don't care; Receive success analyze whether it is Accept; Send failure NULL; Receive timeout SoftRST.
	PD_Prot_pSet( NULL , pProt_RX_SrcCap , NULL , PD_TX_HRST );
}

/*********************************************************************
 * @fn      pProt_RX_SrcCap
 *
 * @brief   Receive SrcCap.
 *
 * @return  none
 */
void pProt_RX_SrcCap(void)
{
	if ( rxHeader->MsgType == PD_Data_SrcCap ) {
		PD_PHY.rxSrcCapCnt = rxHeader->NDO*4;
		memset(PD_PHY.rxSrcCap,0,sizeof(PD_PHY.rxSrcCap));
		memcpy(PD_PHY.rxSrcCap,&PD_RX_BUF[1],PD_PHY.rxSrcCapCnt);
		PD_User_Snk_Rx_SrcCap();
		pProt_TX_Request();
	}
	else pProt_IDLE();
}

/*********************************************************************
 * @fn      pProt_TX_Request
 *
 * @brief   Send Request.
 *
 * @return  none
 */
void pProt_TX_Request(void)
{
	st_Request_Fixed *pRequest = (st_Request_Fixed *)&PD_TX_BUF[1];
	st_SrcCap_Fixed *pSinkCap = (st_SrcCap_Fixed *)&SinkCap[SinkCapCnt-1];
	st_SrcCap_Fixed *pSrcCap;
	u16 CapMismatch=10000;
	pRequest->Data = 0;
	pRequest->ObjectPos = 1;
	for ( u8 i=rxHeader->NDO;i;i--) {		//Traverse SrcCap PDO
		pSrcCap = (st_SrcCap_Fixed *)&PD_PHY.rxSrcCap[i-1];
		if ( pSrcCap->FixedSupply ) continue;									//Not fixed;
		if ( pSrcCap->Voltage > pSinkCap->Voltage ) continue;					//Don't use high voltage
		if ( pSrcCap->Voltage == pSinkCap->Voltage ) {							//Equal voltage direct acceptance
			pRequest->ObjectPos = i;
			break;
		}
		else if ( CapMismatch > ( pSinkCap->Voltage - pSrcCap->Voltage ) ) {	//Record the difference of voltage inequality
			CapMismatch = pSinkCap->Voltage - pSrcCap->Voltage;
			pRequest->ObjectPos = i;
		}
	}
	pSrcCap = (st_SrcCap_Fixed *)&PD_PHY.rxSrcCap[ pRequest->ObjectPos - 1 ];
	pRequest->USBComm = 0;
	pRequest->Current = pRequest->MaxCurrent = pSrcCap->MaxCurrent;
	PD_PHY.savedRequest.Data = pRequest->Data;
	PD_PHY_Header_Init(1,1,PD_Data_Request);
//Accept within 30ms;
	PD_PHY.WaitMsgRx = 1;
	PD_PHY.MsgRxCnt = 28;
//	Send success don't care; Receive success analyze whether it is Accept; Send failure SoftRST; Receive timeout HRST.
	PD_Prot_pSet( NULL , pProt_RX_Accept , pProt_TX_SoftRst , PD_TX_HRST );
}

/*********************************************************************
 * @fn      pProt_RX_Accept
 *
 * @brief   Receive Accept.
 *
 * @return  none
 */
void pProt_RX_Accept(void)
{

//Check if it is Accept
	DEBUG_Print("Check Accept\r\n");
	if ( rxHeader->MsgType == PD_Ctrl_Accept ) {
//Received PS_RDY within 500ms
		PD_PHY.WaitMsgRx = 1;
		PD_PHY.MsgRxCnt = 500;
//	Send success don't care; Receive success analyze whether it is PS_RDY; No send; Receive timeout HRST
		PD_Prot_pSet( NULL , pProt_RX_PS_RDY , NULL , PD_TX_HRST );
	}
	else {
		PD_PHY.WaitMsgRx = 0;
		pProt_TX_SoftRst();
	}
}

/*********************************************************************
 * @fn      pProt_RX_PS_RDY
 *
 * @brief   Receive PS_RDY.
 *
 * @return  none
 */
void pProt_RX_PS_RDY(void)
{
	DEBUG_Print("Check PS_RDY\r\n");
	if ( rxHeader->MsgType == PD_Ctrl_PS_Ready ) {
		DEBUG_Print("ADC VBUS:%.2f\r\n",(float)GetADC_VBUS/4096*3.3/33*233);
		PD_PHY.WaitMsgRx = 0;
		VDM_State.Explicit_Contract_Established = 1;
		PD_User_Snk_Rx_PS_RDY();
		PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );
	}
	else {
		PD_PHY.WaitMsgRx = 0;
		PD_TX_HRST();
	}
}

/*********************************************************************
 * @fn      pProt_TX_SoftRst
 *
 * @brief   Send Software reset.
 *
 * @return  none
 */
void pProt_TX_SoftRst(void)
{
	DEBUG_Print("TX SoftRST\r\n");
//Generate SoftReset
	PD_PHY.TxMsgID = PD_PHY.RxMsgID = 0;
	VDM_State.Explicit_Contract_Established = 0;
	VDM_State.Enter_Mode_already = 0;
	PD_PHY_Header_Init(1,0,PD_Ctrl_SoftReset);
//Accept within 50ms
	PD_PHY.WaitMsgRx = 1;
	PD_PHY.MsgRxCnt = 50;
	PD_Prot_pSet( NULL , pProt_SoftRst_RX_Accept , NULL , PD_TX_HRST );
}

/*********************************************************************
 * @fn      pProt_SoftRst_RX_Accept
 *
 * @brief   Receive Software reset Accept.
 *
 * @return  none
 */
void pProt_SoftRst_RX_Accept(void)
{
	DEBUG_Print("Check Accept\r\n");
	PD_PHY.WaitMsgRx = 0;
	if ( rxHeader->MsgType == PD_Ctrl_Accept ) pProt_Excute_SoftRst();
	else PD_TX_HRST();
}

/*********************************************************************
 * @fn      pProt_RX_SoftRst
 *
 * @brief   Receive Software reset.
 *
 * @return  none
 */
void pProt_RX_SoftRst(void)
{
	DEBUG_Print("Rx SoftRST\r\n");
	VDM_State.Explicit_Contract_Established = 0;
	VDM_State.Enter_Mode_already = 0;
	PD_PHY.TxMsgID = 0;
	PD_PHY_Header_Init(1,0,PD_Ctrl_Accept);
	PD_Prot_pSet( pProt_Excute_SoftRst , pProt_IDLE , NULL , NULL );
}

/*********************************************************************
 * @fn      pProt_Excute_SoftRst
 *
 * @brief   Excute Software reset.
 *
 * @return  none
 */
void pProt_Excute_SoftRst(void)
{
	DEBUG_Print("Excute SoftRst\r\n");
	if ( PD_DEVICE.DevStat ) {
		PD_PHY.SrcCapCnt = 50;
		pProt_TX_SrcCap();
		PD_PHY.MsgTxCnt = 450;
	}
	else pProt_Wait_SrcCap();
}

/*********************************************************************
 * @fn      pProt_RX_ChunkedMsg
 *
 * @brief   Receive ChunkedMsg.
 *
 * @return  none
 */
void pProt_RX_ChunkedMsg(void)
{
	PD_PHY_Header_Init(45,0,PD_Ctrl_NotSupported);
	PD_Prot_pSet( NULL , pProt_IDLE , pProt_TX_SoftRst , NULL );
}
