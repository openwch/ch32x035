/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32x035_usbfs_device.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2023/12/26
* Description        : This file contains all the functions prototypes for the
*                      USBFS firmware library.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#ifndef __CH32X035_USBFS_DEVICE_H_
#define __CH32X035_USBFS_DEVICE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "debug.h"
#include "string.h"
#include "usb_desc.h"
#include <ch32x035_usb.h>
#include "UART.h"

/******************************************************************************/
/* Global Define */
#ifndef __PACKED
  #define __PACKED   __attribute__((packed))
#endif

/* end-point number */
#define DEF_UEP_IN                    0x80
#define DEF_UEP_OUT                   0x00
#define DEF_UEP0                      0x00
#define DEF_UEP1                      0x01
#define DEF_UEP2                      0x02
#define DEF_UEP3                      0x03
#define DEF_UEP4                      0x04
#define DEF_UEP5                      0x05
#define DEF_UEP6                      0x06
#define DEF_UEP7                      0x07
#define DEF_UEP_NUM                   8

#define USBFSD_UEP_MOD_BASE         0x4002340C
#define USBFSD_UEP_DMA_BASE         0x40023410
#define USBFSD_UEP_LEN_BASE         0x40023420
#define USBFSD_UEP_CTL_BASE         0x40023422
#define USBFSD_UEP_RX_EN            0x08
#define USBFSD_UEP_TX_EN            0x04
#define USBFSD_UEP_BUF_MOD          0x01
#define DEF_UEP_DMA_LOAD            0 /* Direct the DMA address to the data to be processed */
#define DEF_UEP_CPY_LOAD            1 /* Use memcpy to move data to a buffer */
#define USBFSD_UEP_MOD( N )         (*((volatile uint8_t *)( USBFSD_UEP_MOD_BASE + N )))
#define USBFSD_UEP_TX_CTRL( N )     (*((volatile uint8_t *)( USBFSD_UEP_CTL_BASE + N * 0x04 )))
#define USBFSD_UEP_RX_CTRL( N )     (*((volatile uint8_t *)( USBFSD_UEP_CTL_BASE + N * 0x04 )))
#define USBFSD_UEP_DMA( N )         (*((volatile uint32_t *)( USBFSD_UEP_DMA_BASE + N * 0x04 )))
#define USBFSD_UEP_BUF( N )         ((uint8_t *)(*((volatile uint32_t *)( USBFSD_UEP_DMA_BASE + N * 0x04 ))) + 0x20000000)
#define USBFSD_UEP_TLEN( N )        (*((volatile uint16_t *)( USBFSD_UEP_LEN_BASE + N * 0x04 )))

#define DEF_UEP_DMA_LOAD              0 /* Direct the DMA address to the data to be processed */
#define DEF_UEP_CPY_LOAD              1 /* Use memcpy to move data to a buffer */

/* Setup Request Packets */
#define pUSBFS_SetupReqPak                 ((PUSB_SETUP_REQ)USBFS_EP0_4Buf)


#define USB_IOEN                    0x00000080
#define USB_PHY_V33                 0x00000040
#define UDP_PUE_MASK                0x0000000C
#define UDP_PUE_DISABLE             0x00000000
#define UDP_PUE_35UA                0x00000004
#define UDP_PUE_10K                 0x00000008
#define UDP_PUE_1K5                 0x0000000C

#define UDM_PUE_MASK                0x00000003
#define UDM_PUE_DISABLE             0x00000000
#define UDM_PUE_35UA                0x00000001
#define UDM_PUE_10K                 0x00000002
#define UDM_PUE_1K5                 0x00000003
/*******************************************************************************/
/* Variable Definition */
/* Global */
extern const    uint8_t  *pUSBFS_Descr;

/* Setup Request */
extern volatile uint8_t  USBFS_SetupReqCode;
extern volatile uint8_t  USBFS_SetupReqType;
extern volatile uint16_t USBFS_SetupReqValue;
extern volatile uint16_t USBFS_SetupReqIndex;
extern volatile uint16_t USBFS_SetupReqLen;

/* USB Device Status */
extern volatile uint8_t  USBFS_DevConfig;
extern volatile uint8_t  USBFS_DevAddr;
extern volatile uint8_t  USBFS_DevSleepStatus;
extern volatile uint8_t  USBFS_DevEnumStatus;

/* Endpoint Buffer */
extern __attribute__ ((aligned(4))) uint8_t USBFS_EP0_4Buf[ ];
extern __attribute__ ((aligned(4))) uint8_t USBFS_EP1_Buf[ ];
extern __attribute__ ((aligned(4))) uint8_t USBFS_EP2_Buf[ ];
extern __attribute__ ((aligned(4))) uint8_t USBFS_EP3_Buf[ ];
/* USB IN Endpoint Busy Flag */
extern volatile uint8_t  USBFS_Endp_Busy[ ];

/******************************************************************************/
/* external functions */
extern void USBFS_Device_Init( FunctionalState sta , PWR_VDD VDD_Voltage);
extern void USBFS_Device_Endp_Init(void);
extern void USBFS_RCC_Init(void);
extern void USBFS_Sleep_Wakeup_Operate(void);
extern uint8_t USBFS_Endp_DataUp(uint8_t endp, uint8_t *pbuf, uint16_t len, uint8_t mod);

#ifdef __cplusplus
}
#endif


#endif /* __CH32X035_USBFS_DEVICE_H_ */

