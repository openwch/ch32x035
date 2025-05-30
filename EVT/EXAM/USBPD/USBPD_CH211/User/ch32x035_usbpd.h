/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32x035_usbpd.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/04/06
 * Description        :
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CH32X035_USBPD_H
#define __CH32X035_USBPD_H

#ifdef __cplusplus
extern "C" {
#endif


// Register Bit Definition
//USBPD->CONFIG
#define PD_FILT_ED          (1<<0)              //PD Filter, 0: OFF, 1: ON
#define PD_ALL_CLR          (1<<1)              //PD mode clears all interrupt flags, 0: invalid, 1: clear interrupt flags
#define CC_SEL              (1<<2)              //Select current PD communication port,0: use CC1 port to communicate,1: use CC2 port to communicate
#define PD_DMA_EN           (1<<3)              //Enable DMA for USBPD, this bit must be set to 1 in normal transfer mode,1: Enable DMA function and DMA interrupt,0: Disable DMA.
#define PD_RST_EN           (1<<4)              //PD mode reset command enable,0: invalid,1: reset
#define WAKE_POLAR          (1<<5)              //PD port wake-up level, 0: active low, 1: active high
#define IE_PD_IO            (1<<10)             //PD IO interrupt enable
#define IE_RX_BIT           (1<<11)             //Receive bit interrupt enable
#define IE_RX_BYTE          (1<<12)             //Receive byte interrupt enable
#define IE_RX_ACT           (1<<13)             //Receive completion interrupt enable
#define IE_RX_RESET         (1<<14)             //Receive reset interrupt enable
#define IE_TX_END           (1<<15)             //End-of-send interrupt enable

//USBPD->CONTROL
#define PD_TX_EN            (1<<0)              //USBPD transceiver mode and transmit enable,0: PD receive enable,1: PD transmit enable
#define BMC_START           (1<<1)              //BMC sends start signal
#define DATA_FLAG           (1<<5)              //Cache data valid flag bit
#define TX_BIT_BACK         (1<<6)              //Indicates the current bit status of the BMC when sending the code,0: idle;,1: indicates that the BMC bytes are being sent
#define BMC_BYTE_HI         (1<<7)              //Indicates the current half-byte status of the PD data being sent and received, 0: the lower 4 bits are being processed, 1: the upper 4 bits are being processed

//USBPD->TX_SEL
#define TX_SEL1             (0<<0)
#define TX_SEL1_SYNC1       (0<<0)              //0 £ºSYNC1
#define TX_SEL1_RST1        (1<<0)              //1 £ºRST1
#define TX_SEL2             (0<<2)
#define TX_SEL2_SYNC1       (0<<2)              //00:SYNC1
#define TX_SEL2_SYNC3       (1<<2)              //01:SYNC3
#define TX_SEL2_RST1        (2<<2)              //1x:RST1
#define TX_SEL3             (0<<4)
#define TX_SEL3_SYNC1       (0<<4)              //00:SYNC1
#define TX_SEL3_SYNC3       (1<<4)              //01:SYNC3
#define TX_SEL3_RST1        (2<<4)              //1x:RST1
#define TX_SEL4             (0<<6)
#define TX_SEL4_SYNC2       (0<<6)              //00:SYNC2
#define TX_SEL4_SYNC3       (1<<6)              //01:SYNC3
#define TX_SEL4_RST2        (2<<6)              //1x:RST2

//USBPD->STATUS
#define BMC_AUX             (3<<0)              //BMC auxiliary information, when doing receive SOP:,when doing send CRC: CRC byte counter
#define BMC_AUX_INVALID     (0<<0)              //00: not valid
#define BMC_AUX_SOP0        (1<<0)              //01£ºSOP0
#define BMC_AUX_SOP1_HRST   (2<<0)              //10£ºSOP1 hard reset
#define BMC_AUX_SOP2_CRST   (3<<0)              //11£ºSOP2 cable reset
#define BUF_ERR             (1<<2)              //BUFFER or DMA error interrupt flag, write 1 to clear 0, write 0 to void
#define IF_RX_BIT           (1<<3)              //Receive bit or 5bit interrupt flag, write 1 to clear 0, write 0 to void
#define IF_RX_BYTE          (1<<4)              //Receive byte or SOP interrupt flag, write 1 to clear 0, write 0 to void
#define IF_RX_ACT           (1<<5)              //Receive completion interrupt flag, write 1 to clear 0, write 0 to void
#define IF_RX_RESET         (1<<6)              //Receive reset interrupt flag, write 1 to clear 0, write 0 to void
#define IF_TX_END           (1<<7)              //Transfer completion interrupt flag, write 1 to clear 0, write 0 to void

//USBPD->PORT_CC1
//USBPD->PORT_CC2
#define PA_CC_AI            (1<<0)              //CC1 port comparator analogue input
#define CC_PD               (1<<1)              //CC1 port down resistor enable,0: disable pull down resistor ,1: enable 5.1K¦¸ pull down resistor
#define CC_PU_CLR           (3<<2)              //CC1 port pull-up current selection
#define CC_NO_PU            (0<<2)              //00: Pull-up current forbidden
#define CC_PU_330           (1<<2)              //01£º330uA
#define CC_PU_180           (2<<2)              //10£º180uA
#define CC_PU_80            (3<<2)              //11: 80uA
#define CC_LVO              (1<<4)              //CC1 port output low voltage enable,0: normal voltage VDD weak drive output,1: low voltage drive output
#define CC_CE               (7<<5)              //Enable of voltage comparator on port /CC1,001: Reserved
#define CC_NO_CMP           (0<<5)              //000: closed
#define CC_CMP_22           (2<<5)              //010£º0.22V
#define CC_CMP_45           (3<<5)              //011£º0.45V
#define CC_CMP_55           (4<<5)              //100£º0.55V
#define CC_CMP_66           (5<<5)              //101£º0.66V
#define CC_CMP_95           (6<<5)              //110£º0.95V
#define CC_CMP_123          (7<<5)              //111£º1.23V


#define USBPD_IN_HVT       (1<<9)
/*********************************************************
* PD pin PC14/PC15 high threshold input mode:
* 1: High threshold input, ~2.2V typical, reduces PD pass
* I/O power consumption during signalling;
* 0: Normal GPIO threshold input. *
* *******************************************************/
#define USBPD_PHY_V33     (1<<8)
/**********************************************************
* PD transceiver PHY pull-up limit configuration bits:
* 1: direct VDD, output voltage up to VDD, for VDD
* for applications with 3.3V;
* 0: LDO buck enabled, limited to approx. 3.3V, for applications with VDD
* applications with more than 4V.
* ********************************************************/

#define		PD_PHY_HRST			        0xFF              /* Send By: Sink */

#define		PD_Ctrl_Reserved			0x00
#define		PD_Ctrl_GoodCRC				0x01
#define		PD_Ctrl_GotoMin				0x02
#define		PD_Ctrl_Accept				0x03
#define		PD_Ctrl_Reject				0x04
#define		PD_Ctrl_Ping				0x05
#define		PD_Ctrl_PS_Ready			0x06
#define		PD_Ctrl_GetSrcCap			0x07
#define		PD_Ctrl_GetSinkCap			0x08
#define		PD_Ctrl_DRSwap				0x09
#define		PD_Ctrl_PRSwap				0x0A
#define		PD_Ctrl_VconnSwap			0x0B
#define		PD_Ctrl_Wait				0x0C
#define		PD_Ctrl_SoftReset			0x0D
#define		PD_Ctrl_DataReset			0x0E
#define		PD_Ctrl_DataResetComplete	0x0F
#define		PD_Ctrl_NotSupported		0x10
#define		PD_Ctrl_GetSrcCapExtended	0x11
#define		PD_Ctrl_GetStatus			0x12
#define		PD_Ctrl_FRSwap				0x13
#define		PD_Ctrl_GetPPSStatus		0x14
#define		PD_Ctrl_GetCountryCodes		0x15
#define		PD_Ctrl_GetSinkCapExtended	0x16
#define		PD_Ctrl_GetSrcInfo			0x17
#define		PD_Ctrl_GetRevision			0x18

#define		PD_Data_Reserved1			0x00
#define		PD_Data_SrcCap				0x01
#define		PD_Data_Request				0x02
#define		PD_Data_BIST				0x03
#define		PD_Data_SinkCap				0x04
#define		PD_Data_BatteryStatus		0x05
#define		PD_Data_Alert				0x06
#define		PD_Data_GetCountryInfo		0x07
#define		PD_Data_EnterUSB			0x08
#define		PD_Data_EPRRequest			0x09
#define		PD_Data_EPRMode				0x0A
#define		PD_Data_SrcInfo				0x0B
#define		PD_Data_Revision			0x0C
#define		PD_Data_Reserved2			0x0D
#define		PD_Data_Reserved3			0x0E
#define		PD_Data_VendorDefined		0x0F

#define		PD_Ext_Reserved				0x00
#define		PD_Ext_SrcCapExtended		0x01
#define		PD_Ext_Status				0x02
#define		PD_Ext_GetBatteryCap		0x03
#define		PD_Ext_GetBatteryStatus		0x04
#define		PD_Ext_BattertCap			0x05
#define		PD_Ext_GetManufacturerInfo	0x06
#define		PD_Ext_ManufacturerInfo		0x07
#define		PD_Ext_SecurityRequest		0x08
#define		PD_Ext_SecurityResponse		0x09
#define		PD_Ext_FWUpdateRequest		0x0A
#define		PD_Ext_FWUpdateResponse		0x0B
#define		PD_Ext_PPSStatus			0x0C
#define		PD_Ext_SinkCapExtended		0x0F

#define		PD_VDM_DiscoverIdentity		0x01
#define		PD_VDM_DiscoverSVIDs		0x02
#define		PD_VDM_DiscoverModes		0x03
#define		PD_VDM_EnterModes			0x04
#define		PD_VDM_ExitModes			0x05
#define		PD_VDM_Attention			0x06

#define 	DEF_Unstructured_VDM		 0
#define 	DEF_Structured_VDM			 1

/* PD Revision */
#define PD_Rev2         		        0x01
#define PD_Rev3         		        0x02

#define DevRole_Sink			         0
#define DevRole_Src				         1
#define DevRole_DRP				         2
#define DevRole_DRP_TrySink		         3
#define DevRole_DRP_TrySrc		         4

#ifdef __cplusplus
}
#endif

#endif
