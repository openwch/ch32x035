/********************************** (C) COPYRIGHT  *******************************
 * File Name          : libCH32_USBPD.h
 * Author             : WCH
 * Version            : V1.0.1
 * Date               : 2025/10/27
 * Description        :
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "debug.h"

#ifndef USBPD_LIBCH32_USBPD_H_
#define USBPD_LIBCH32_USBPD_H_

#define PD_Vconn_CC1                    CH211_I2C_SetBit(CC_CTRL, CC1_VCE, 0)
#define PD_Float_CC1                    CH211_I2C_SetBit(CC_CTRL, 0, CC1_VCE)

#define PD_Vconn_CC2                    CH211_I2C_SetBit(CC_CTRL, CC2_VCE, 0)
#define PD_Float_CC2                    CH211_I2C_SetBit(CC_CTRL, 0, CC2_VCE)

#define PD_Enable_Discharge             CH211_I2C_SetBit(HVCP_CTRL, VBUS_DISC, 0)
#define PD_Disable_Discharge            CH211_I2C_SetBit(HVCP_CTRL, 0, VBUS_DISC)

#define PD_Enable_Rd                    CH211_I2C_SetBit(CC_CTRL, CC1_PD|CC2_PD, 0)
#define PD_Disable_Rd                   CH211_I2C_SetBit(CC_CTRL, 0, CC1_PD|CC2_PD)

//#define GetADC_VBUS                     ADC_Get_Sample(ADC_Channel_5)

#define Check_CtrlMsg_Type_Reserved		rxHeader->MsgType > ( (PD_PHY.Header.SpecRevision==PD_Rev3)?(sizeof(enum_Idle_CtrlMsg)-1):(PD_Ctrl_SoftReset) )
#define Check_DataMsg_Type_Reserved		( rxHeader->MsgType > ( (PD_PHY.Header.SpecRevision==PD_Rev3)?(sizeof(enum_Idle_DataMsg)-1):(PD_Data_SinkCap) ) ) && ( rxHeader->MsgType != PD_Data_VendorDefined )

typedef struct {
	union {
		u16 Data;
		struct {
			u16 MsgType:5;
			u16 PortDataRole:1;
			u16 SpecRevision:2;
			u16 PortPwrRole:1;
			u16 MsgID:3;
			u16 NDO:3;
			u16 Extended:1;
		};
	};
} st_Prot_Header;

typedef struct {
	union {
		u16 Data;
		struct {
			u16 DataSize:9;
			u16 Reserved:1;
			u16 RequestChunk:1;
			u16 ChunkNumber:4;
			u16 Chunked:1;
		};
	};
} st_Extended_Header;

typedef struct {
	union {
		u32 Data;
		struct {
			u32 MaxCurrent:10;
			u32 Current:10;
			u32 Reserved:2;
			u32 EPRMode:1;
			u32 UnchunkedExtended:1;
			u32 NoUSBSuspend:1;
			u32 USBComm:1;
			u32 CapbilityMismatch:1;
			u32 GiveBack:1;
			u32 ObjectPos:4;
		};
	};
} st_Request_Fixed;

typedef struct {

	void (*pDevChk)();
	void (*pUserUnattached)();		//Connection successful call user program
	void (*pUserAttached)();		//Connect remove call user program

	u8 DevRole;		                //Sink   Src   DRP   DRP.TrySink   DRP.TrySrc
	u8 DevStat;		                //0:Sink   1:Src

	u8 ConnectStat;
	u8 VconnStat;			        //VCONN needs to read eMarker;

	u16 Cnt;
	u16 TryCnt;
	u16 Timeout;

	uint16_t *PortCC;		        //Port control register used by CC
	uint32_t GpioCC;	        	//GPIO of CC used

	uint16_t *PortVconn;		    //Port control register used by CC
	uint32_t GpioVconn;		        //GPIO of CC used

} st_PD_DEVICE;

typedef struct {

									//	SOP		SOP'	SOP"	HRST	CRST
	u8 RxSop;	                    //  01		10		11		10		11
	u8 TxSop;	                    //	0x00	0x50	0x44	0xFF	0x73


	u8 TxMsgID;
	u8 RxMsgID;
	u8 RetryCnt;
	u8 PDExist;				        //The first message does not check the MsgID.
	u16 IdleCnt;				    //PD Idle count


	u8 SrcCapCnt;
	st_Prot_Header Header;		    //Current header default values, including DRP status

	u8 WaitTxGcrc;
	u8 WaitRxGcrc;

	u8 WaitMsgTx;		            //Waiting to send message flag
	u16 MsgTxCnt;		            //Message sending timing (ms)

	u8 WaitMsgRx;		            //Waiting to receive message flag
	u16 MsgRxCnt;		            //Message receive timeout (ms)

	u8 VoltChanging;		        //Regulation flag
	u8 tSrcTransition;		        //Waiting to start pressure regulation 	25-35
	u16 tPSTransition;		        //Regulating stage		450-550

	void (*pRxFinish)();		    //Received successfully: Message received and replied GoodCRC
	void (*pRxTimeout)();		    //Receive timeout: failed to receive the message within the specified time, not judging the type of message received.
	void (*pTxFinish)();		    //Sent successfully: The message was sent successfully and has been received with GoodCRC.
	void (*pTxTimeout)();		    //Send timeout: failed to receive the correct GoodCRC within the retry limit
	void (*pRxHRST)();			    //Received HRST, and sent HRST

	u32 rxSrcCap[7];
	u8  rxSrcCapCnt;
	st_Request_Fixed savedRequest;

} st_PD_PHY;

extern u16 vSafe0V ;                 //0.8V
extern u16 vSinkDisconnect ;         //3.67V
extern u16 vVBUSon;                  //2.27V
extern u16 GetADC_VBUS;

void Set_DevChk(u8 Role);
void PD_Lib_Check_Version(void);

u16 ADC_Get_Sample(uint8_t Channel);
void PD_Init(void);
void PD_PHY_Reset_Value(u8 Role);
void PD_PHY_Header_Init(u8 cnt, u8 NDO, u8 MsgType);
void PD_Prot_pSet( void (*pTxFinish) , void (*pRxFinish) ,void (*pTxTimeout) , void (*pRxTimeout) );
void PD_RX_Reserved(void);
void PD_RX_VDM_Reserved(void);

void PD_TX_HRST(void);
void PD_TX_BIST(void);
void PD_ENABLE_BIST(void);
extern void BIST_Output();
void PD_TX_NotSupported(void);
void PD_TX_Reject(void);
void PD_TX_ManufacturerInfo(void);
void PD_TX_Revision(void);
void PD_TX_SrcInfo(void);
void PD_TX_SinkCapExt(void);
void PD_TX_ChunkRequest(void);

void PD_TX_VCONN_DISC_IDENT(void);

void PD_RX_HRST(void);
void Prot_NULL(void);
extern __attribute__ ((aligned(4))) uint16_t PD_RX_BUF[ 28 ];		//Receiving Buffer
extern __attribute__ ((aligned(4))) uint16_t PD_TX_BUF[ 28 ];		//Sending Buffer
extern st_Prot_Header *rxHeader;
extern st_Prot_Header *txHeader;
extern st_Extended_Header *rxExtHeader;
extern st_PD_PHY PD_PHY;
extern st_PD_DEVICE PD_DEVICE;
extern u32 SrcCap[7];
extern vu8 SrcCapCnt;
extern u32 SinkCap[7];
extern vu8 SinkCapCnt;
#endif /* USBPD_LIBCH32_USBPD_H_ */
