/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32x035_usbfs_device.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2024/03/07
* Description        : This file provides all the USBFS firmware functions.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#include <ch32x035_usbfs_device.h>
#include "iap.h"

/*******************************************************************************/
/* Variable Definition */
/* USB_Device_clock_source */

/* Global */
const    uint8_t  *pUSBFS_Descr;
#define Version_Num   0x0100   //V0100
#define DevEP0SIZE    0x40
u8 EP2_Tx_Buffer[2];

/* Setup Request */
volatile uint8_t  USBFS_SetupReqCode;
volatile uint8_t  USBFS_SetupReqType;
volatile uint16_t USBFS_SetupReqValue;
volatile uint16_t USBFS_SetupReqIndex;
volatile uint16_t USBFS_SetupReqLen;

/* USB Device Status */
volatile uint8_t  USBFS_DevConfig;
volatile uint8_t  USBFS_DevAddr;
volatile uint8_t  USBFS_DevSleepStatus;
volatile uint8_t  USBFS_DevEnumStatus;

/* Endpoint Buffer */
__attribute__ ((aligned(4))) uint8_t EP0_DatabufHD[64]; //ep0(64)
__attribute__ ((aligned(4))) uint8_t EP2_DatabufHD[64+64];  //ep2_out(64)+ep2_in(64)

uint8_t* pEP0_RAM_Addr;                       //ep0(64)
uint8_t* pEP2_RAM_Addr;                       //ep2_out(64)+ep2_in(64)

/******************************************************************************/
/* Device Descriptor */
const uint8_t MyDevDescrHD[] = { 0x12, 0x01, 0x10, 0x01, 0xFF, 0x80, 0x55,
        DevEP0SIZE, 0x48, 0x43, 0xe0, 0x55,  //USB MODULE
        (u8) Version_Num, (u8) (Version_Num >> 8),
        0x00, 0x00, 0x00, 0x01 };

/* Configration Descriptor */
const uint8_t MyCfgDescrHD[] = { 0x09, 0x02, 0x20, 0x00, 0x01, 0x01, 0x00, 0x80,
        0x32, 0x09, 0x04, 0x00, 0x00, 0x02, 0xFF, 0x80, 0x55, 0x00, 0x07, 0x05,
        0x82, 0x02, 0x40, 0x00, 0x00, 0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x00 };

/* USB IN Endpoint Busy Flag */
volatile uint8_t  USBFS_Endp_Busy[ DEF_UEP_NUM ];


/******************************************************************************/
/* Interrupt Service Routine Declaration*/
void USBFS_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));


/*********************************************************************
 * @fn      USBFS_RCC_Init
 *
 * @brief   Initializes the USBFS clock configuration.
 *
 * @return  none
 */
void USBFS_RCC_Init(void)
{
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE );
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_USBFS, ENABLE );
}

/*********************************************************************
 * @fn      USBFS_Device_Endp_Init
 *
 * @brief   Initializes USB device endpoints.
 *
 * @return  none
 */
void USBFS_Device_Endp_Init( void )
{
    uint8_t i;

    pEP0_RAM_Addr = EP0_DatabufHD;
    pEP2_RAM_Addr = EP2_DatabufHD;

    USBFSD->UEP2_3_MOD = USBFS_UEP2_TX_EN|USBFS_UEP2_RX_EN;
    USBFSD->UEP0_DMA = (uint32_t)pEP0_RAM_Addr;
    USBFSD->UEP2_DMA = (uint32_t)pEP2_RAM_Addr;

    USBFSD->UEP0_CTRL_H = USBFS_UEP_R_RES_ACK | USBFS_UEP_T_RES_NAK;
    USBFSD->UEP1_CTRL_H = USBFS_UEP_R_RES_NAK | USBFS_UEP_T_RES_ACK | USBFS_UEP_T_AUTO_TOG;
    USBFSD->UEP2_CTRL_H = USBFS_UEP_R_RES_ACK;
    USBFSD->UEP3_CTRL_H = USBFS_UEP_R_RES_NAK;
    USBFSD->UEP4_CTRL_H = USBFS_UEP_R_RES_NAK;
    USBFSD->UEP5_CTRL_H = USBFS_UEP_R_RES_NAK;
    USBFSD->UEP6_CTRL_H = USBFS_UEP_R_RES_NAK;
    USBFSD->UEP7_CTRL_H = USBFS_UEP_R_RES_NAK;

    USBFSD->UEP2_TX_LEN = 8;

    /* Clear End-points Busy Status */
    for( i=0; i<DEF_UEP_NUM; i++ )
    {
        USBFS_Endp_Busy[ i ] = 0;
    }
}


/*********************************************************************
 * @fn      GPIO_USB_INIT
 *
 * @brief   Initializes USB GPIO.
 *
 * @return  none
 */
void GPIO_USB_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_16;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_17;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/*********************************************************************
 * @fn      USBFS_Device_Init
 *
 * @brief   Initializes USB device.
 *
 * @return  none
 */
void USBFS_Device_Init( FunctionalState sta ,PWR_VDD VDD_Voltage)
{
    if( sta )
    {
        GPIO_USB_INIT();
        if( VDD_Voltage == PWR_VDD_5V )
        {
            AFIO->CTLR = (AFIO->CTLR & ~(UDP_PUE_MASK | UDM_PUE_MASK | USB_PHY_V33)) | UDP_PUE_10K | USB_IOEN;
        }
        else
        {
            AFIO->CTLR = (AFIO->CTLR & ~(UDP_PUE_MASK | UDM_PUE_MASK )) | USB_PHY_V33 | UDP_PUE_1K5 | USB_IOEN;
        }
        USBFSD->BASE_CTRL = 0x00;
        USBFS_Device_Endp_Init( );
        USBFSD->DEV_ADDR = 0x00;
        USBFSD->BASE_CTRL = USBFS_UC_DEV_PU_EN | USBFS_UC_INT_BUSY | USBFS_UC_DMA_EN;
        USBFSD->INT_FG = 0xff;
        USBFSD->UDEV_CTRL = USBFS_UD_PD_DIS | USBFS_UD_PORT_EN;
        USBFSD->INT_EN = USBFS_UIE_SUSPEND | USBFS_UIE_BUS_RST | USBFS_UIE_TRANSFER;
        NVIC_EnableIRQ( USBFS_IRQn );
    }
    else
    {
        AFIO->CTLR = AFIO->CTLR & ~(UDP_PUE_MASK | UDM_PUE_MASK | USB_IOEN);
        USBFSD->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL;
        Delay_Us( 10 );
        USBFSD->BASE_CTRL = 0x00;
        NVIC_DisableIRQ( USBFS_IRQn );
    }
}

/*********************************************************************
 * @fn      DevEP2_IN_Deal
 *
 * @brief   Device endpoint2 IN.
 *
 * @param   l - IN length(<64B)
 *
 * @return  none
 */
void DevEP2_IN_Deal( uint8_t l )
{
    USBFSD->UEP2_TX_LEN = l;
    USBFSD->UEP2_CTRL_H = (USBFSD->UEP2_CTRL_H & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_ACK;
}

/*********************************************************************
 * @fn      DevEP2_OUT_Deal
 *
 * @brief   Deal device Endpoint 2 OUT.
 *
 * @param   l - Data length.
 *
 * @return  none
 */
void DevEP2_OUT_Deal( uint8_t l )
{
    uint8_t s ,EP2_Tx_Cnt;

    memcpy(EP2_Rx_Buffer, pEP2_OUT_DataBuf, l);
    s = RecData_Deal();
    if (s != ERR_End) {
        EP2_Tx_Buffer[0] = 0x00;
        if (s == ERR_ERROR)
        {
            pEP2_IN_DataBuf[1] = 0x01;
        }
        else
            pEP2_IN_DataBuf[1] = 0x00;
        EP2_Tx_Cnt = 2;
        DevEP2_IN_Deal(EP2_Tx_Cnt);
    }
}

/*********************************************************************
 * @fn      USBFS_IRQHandler
 *
 * @brief   This function handles HD-FS exception.
 *
 * @return  none
 */
void USBFS_IRQHandler( void )
{

    uint8_t  intflag, intst, errflag;
    uint16_t len;

    intflag = USBFSD->INT_FG;
    intst = USBFSD->INT_ST;

    if( intflag & USBFS_UIF_TRANSFER )
    {
        switch (intst & USBFS_UIS_TOKEN_MASK)
        {
            /* data-in stage processing */
            case USBFS_UIS_TOKEN_IN:
                switch ( intst & ( USBFS_UIS_TOKEN_MASK | USBFS_UIS_ENDP_MASK ) )
                {
                    /* end-point 0 data in interrupt */
                    case USBFS_UIS_TOKEN_IN | DEF_UEP0:
                        if( USBFS_SetupReqLen == 0 )
                        {
                            USBFSD->UEP0_CTRL_H = USBFS_UEP_R_TOG | USBFS_UEP_R_RES_ACK;
                        }
                        if ( ( USBFS_SetupReqType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
                        {
                            /* Non-standard request endpoint 0 Data upload */
                        }
                        else
                        {
                            /* Standard request endpoint 0 Data upload */
                            switch( USBFS_SetupReqCode )
                            {
                                case USB_GET_DESCRIPTOR:
                                        len = USBFS_SetupReqLen >= DevEP0SIZE ? DevEP0SIZE : USBFS_SetupReqLen;
                                        memcpy( pEP0_DataBuf, pUSBFS_Descr, len );
                                        USBFS_SetupReqLen -= len;
                                        pUSBFS_Descr += len;
                                        USBFSD->UEP0_TX_LEN = len;
                                        USBFSD->UEP0_CTRL_H ^= USBFS_UEP_T_TOG;
                                        break;

                                case USB_SET_ADDRESS:
                                        USBFSD->DEV_ADDR = (USBFSD->DEV_ADDR & USBFS_UDA_GP_BIT) | USBFS_DevAddr;
                                        USBFSD->UEP0_CTRL_H = USBFS_UEP_T_RES_NAK | USBFS_UEP_R_RES_ACK;

                                        break;

                                default:
                                        USBFSD->UEP0_TX_LEN = 0;
                                        USBFSD->UEP0_CTRL_H = USBFS_UEP_T_RES_NAK | USBFS_UEP_R_RES_ACK;

                                        break;
                            }
                        }
                        break;

                    /* end-point 2 data in interrupt */
                    case ( USBFS_UIS_TOKEN_IN | DEF_UEP2 ):
                        USBFSD->UEP2_CTRL_H ^= USBFS_UEP_T_TOG;
                        USBFSD->UEP2_CTRL_H = (USBFSD->UEP2_CTRL_H & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
                        USBFS_Endp_Busy[ DEF_UEP2 ] = 0;
                        break;

                    default :
                        break;
                }
                break;

            /* data-out stage processing */
            case USBFS_UIS_TOKEN_OUT:
                switch ( intst & ( USBFS_UIS_TOKEN_MASK | USBFS_UIS_ENDP_MASK ) )
                {
                    /* end-point 0 data out interrupt */
                    case USBFS_UIS_TOKEN_OUT | DEF_UEP0:
                        len = USBFSD->RX_LEN;
                        if ( intst & USBFS_UIS_TOG_OK )
                        {
                            if ( ( USBFS_SetupReqType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
                            {
                                /* Non-standard request end-point 0 Data download */
                                /* Add your code here */
                            }
                            else
                            {
                                /* Standard request end-point 0 Data download */
                                /* Add your code here */
                            }
                            if( USBFS_SetupReqLen == 0 )
                            {
                                USBFSD->UEP0_TX_LEN  = 0;
                                USBFSD->UEP0_CTRL_H = USBFS_UEP_T_TOG | USBFS_UEP_T_RES_ACK;
                            }
                        }
                        break;

                    /* end-point 2 data out interrupt */
                    case USBFS_UIS_TOKEN_OUT | DEF_UEP2:
                        if ( intst & USBFS_UIS_TOG_OK )
                        {
                            /* Write In Buffer */
                            USBFSD->UEP2_CTRL_H ^= USBFS_UEP_R_TOG;
                            len = USBFSD->RX_LEN;
                            DevEP2_OUT_Deal( len );
                        }
                        break;

                }
                break;

            /* Setup stage processing */
            case USBFS_UIS_TOKEN_SETUP:
                USBFSD->UEP0_CTRL_H = USBFS_UEP_T_TOG|USBFS_UEP_T_RES_NAK|USBFS_UEP_R_TOG|USBFS_UEP_R_RES_ACK;

                /* Store All Setup Values */
                USBFS_SetupReqType  = pUSBFS_SetupReqPak->bRequestType;
                USBFS_SetupReqCode  = pUSBFS_SetupReqPak->bRequest;
                USBFS_SetupReqLen   = pUSBFS_SetupReqPak->wLength;
                USBFS_SetupReqValue = pUSBFS_SetupReqPak->wValue;
                USBFS_SetupReqIndex = pUSBFS_SetupReqPak->wIndex;
                len = 0;
                errflag = 0;
                if ( ( USBFS_SetupReqType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
                {
                    /* usb non-standard request processing */
                    /* errflag = 0xFF; if this request or cmd dose not support */
                    errflag = 0xFF;
                }
                else
                {
                    /* usb standard request processing */
                    switch( USBFS_SetupReqCode )
                    {
                        /* get device/configuration/string/report/... descriptors */
                        case USB_GET_DESCRIPTOR:
                            switch( (uint8_t)( USBFS_SetupReqValue >> 8 ) )
                            {
                                /* get usb device descriptor */
                                case USB_DESCR_TYP_DEVICE:
                                    pUSBFS_Descr = MyDevDescrHD;
                                    len = MyDevDescrHD[0];
                                    break;

                                /* get usb configuration descriptor */
                                case USB_DESCR_TYP_CONFIG:
                                    pUSBFS_Descr = MyCfgDescrHD;
                                    len = MyCfgDescrHD[2];
                                    break;

                                /* get usb string descriptor */
                                case USB_DESCR_TYP_STRING:
                                    switch( (uint8_t)( USBFS_SetupReqValue & 0xFF ) )
                                    {
                                        /* Descriptor 0, Language descriptor */
                                        case DEF_STRING_DESC_LANG:

                                            break;

                                        /* Descriptor 1, Manufacturers String descriptor */
                                        case DEF_STRING_DESC_MANU:

                                            break;

                                        /* Descriptor 2, Product String descriptor */
                                        case DEF_STRING_DESC_PROD:

                                            break;

                                        /* Descriptor 3, Serial-number String descriptor */
                                        case DEF_STRING_DESC_SERN:

                                            break;

                                        default:
                                            errflag = 0xFF;
                                            break;
                                    }
                                    break;

                                default :
                                    errflag = 0xFF;
                                    break;
                            }

                            /* Copy Descriptors to Endp0 DMA buffer */
                            if( USBFS_SetupReqLen>len )
                            {
                                USBFS_SetupReqLen = len;
                            }
                            len = (USBFS_SetupReqLen >= DevEP0SIZE) ? DevEP0SIZE : USBFS_SetupReqLen;
                            memcpy( pEP0_DataBuf, pUSBFS_Descr, len );
                            pUSBFS_Descr += len;
                            break;

                        /* Set usb address */
                        case USB_SET_ADDRESS:
                            USBFS_DevAddr = (uint8_t)( USBFS_SetupReqValue & 0xFF );
                            break;

                        /* Get usb configuration now set */
                        case USB_GET_CONFIGURATION:
                            pEP0_DataBuf[0] = USBFS_DevConfig;
                            if ( USBFS_SetupReqLen > 1 )
                            {
                                USBFS_SetupReqLen = 1;
                            }
                            break;

                        /* Set usb configuration to use */
                        case USB_SET_CONFIGURATION:
                            USBFS_DevConfig = (uint8_t)( USBFS_SetupReqValue & 0xFF );
                            USBFS_DevEnumStatus = 0x01;
                            break;

                        /* Clear or disable one usb feature */
                        case USB_CLEAR_FEATURE:
                            if ( ( USBFS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
                            {
                                /* clear one device feature */
                                if( (uint8_t)( USBFS_SetupReqValue & 0xFF ) == USB_REQ_FEAT_REMOTE_WAKEUP )
                                {
                                    /* clear usb sleep status, device not prepare to sleep */
                                    USBFS_DevSleepStatus &= ~0x01;
                                }
                            }
                            else if( ( USBFS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
                            {
                                /* Clear End-point Feature */
                                if( (uint8_t)( USBFS_SetupReqValue & 0xFF ) == USB_REQ_FEAT_ENDP_HALT )
                                {
                                    switch( (uint8_t)( USBFS_SetupReqIndex & 0xFF ) )
                                    {
                                        case ( DEF_UEP_OUT | DEF_UEP1 ):
                                            /* Set End-point 1 OUT ACK*/
                                            USBFSD->UEP1_CTRL_H = USBFS_UEP_R_RES_ACK;
                                            break;

                                        case ( DEF_UEP_OUT | DEF_UEP2 ):
                                            /* Set End-point 2 OUT ACK*/
                                            USBFSD->UEP2_CTRL_H = USBFS_UEP_R_RES_ACK;
                                            break;

                                        case ( DEF_UEP_IN | DEF_UEP1 ):
                                            /* Set End-point 1 IN NAK */
                                            USBFSD->UEP1_CTRL_H = USBFS_UEP_T_RES_NAK;
                                            break;

                                        case ( DEF_UEP_IN | DEF_UEP2 ):
                                            /* Set End-point 2 IN NAK */
                                            USBFSD->UEP2_CTRL_H = USBFS_UEP_T_RES_NAK;
                                            break;

                                        default:
                                            errflag = 0xFF;
                                            break;
                                    }
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                            }
                            else
                            {
                                errflag = 0xFF;
                            }
                            break;

                        /* set or enable one usb feature */
                        case USB_SET_FEATURE:
                            if( ( USBFS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
                            {
                                /* Set Device Feature */
                                if( (uint8_t)( USBFS_SetupReqValue & 0xFF ) == USB_REQ_FEAT_REMOTE_WAKEUP )
                                {
                                    if( MyCfgDescrHD[ 7 ] & 0x20 )
                                    {
                                        /* Set Wake-up flag, device prepare to sleep */
                                        USBFS_DevSleepStatus |= 0x01;
                                    }
                                    else
                                    {
                                        errflag = 0xFF;
                                    }
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                            }
                            else if( ( USBFS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
                            {
                                /* Set End-point Feature */
                                if( (uint8_t)( USBFS_SetupReqValue & 0xFF ) == USB_REQ_FEAT_ENDP_HALT )
                                {
                                    /* Set end-points status stall */
                                    switch( (uint8_t)( USBFS_SetupReqIndex & 0xFF ) )
                                    {
                                        case ( DEF_UEP_OUT | DEF_UEP2 ):
                                            /* Set End-point 2 OUT STALL */
                                            USBFSD->UEP2_CTRL_H = ( USBFSD->UEP2_CTRL_H & ~USBFS_UEP_R_RES_MASK ) | USBFS_UEP_R_RES_STALL;
                                            break;

                                        case ( DEF_UEP_IN | DEF_UEP2 ):
                                            /* Set End-point 2 IN STALL */
                                            USBFSD->UEP2_CTRL_H = ( USBFSD->UEP2_CTRL_H & ~USBFS_UEP_T_RES_MASK ) | USBFS_UEP_T_RES_STALL;
                                            break;

                                        default:
                                            errflag = 0xFF;
                                            break;
                                    }
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                            }
                            else
                            {
                                errflag = 0xFF;
                            }
                            break;

                        /* This request allows the host to select another setting for the specified interface  */
                        case USB_GET_INTERFACE:
                            EP0_DatabufHD[0] = 0x00;
                            if ( USBFS_SetupReqLen > 1 )
                            {
                                USBFS_SetupReqLen = 1;
                            }
                            break;

                        case USB_SET_INTERFACE:
                            EP0_DatabufHD[ 0 ] = 0x00;
                            if( USBFS_SetupReqLen > 1 )
                            {
                                USBFS_SetupReqLen = 1;
                            }

                            break;

                        /* host get status of specified device/interface/end-points */
                        case USB_GET_STATUS:
                            EP0_DatabufHD[ 0 ] = 0x00;
                            EP0_DatabufHD[ 1 ] = 0x00;
                            if ( ( USBFS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
                            {
                                if( USBFS_DevSleepStatus & 0x01 )
                                {
                                    EP0_DatabufHD[ 0 ] = 0x02;
                                }
                            }
                            else if( ( USBFS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
                            {
                                switch( (uint8_t)( USBFS_SetupReqIndex & 0xFF ) )
                                {
                                    case ( DEF_UEP_OUT | DEF_UEP1 ):
                                        if( ( (USBFSD->UEP1_CTRL_H) & USBFS_UEP_R_RES_MASK ) == USBFS_UEP_R_RES_STALL )
                                        {
                                            EP0_DatabufHD[ 0 ] = 0x01;
                                        }
                                        break;

                                    case ( DEF_UEP_IN | DEF_UEP1 ):
                                        if( ( (USBFSD->UEP1_CTRL_H) & USBFS_UEP_T_RES_MASK ) == USBFS_UEP_T_RES_STALL )
                                        {
                                            EP0_DatabufHD[ 0 ] = 0x01;
                                        }
                                        break;

                                    case ( DEF_UEP_IN | DEF_UEP2 ):
                                        if( ( (USBFSD->UEP2_CTRL_H) & USBFS_UEP_T_RES_MASK ) == USBFS_UEP_T_RES_STALL )
                                        {
                                            EP0_DatabufHD[ 0 ] = 0x01;
                                        }
                                        break;

                                    case ( DEF_UEP_OUT | DEF_UEP2 ):
                                        if( ( (USBFSD->UEP2_CTRL_H) & USBFS_UEP_R_RES_MASK ) == USBFS_UEP_R_RES_STALL )
                                        {
                                            EP0_DatabufHD[ 0 ] = 0x01;
                                        }
                                        break;



                                    default:
                                        errflag = 0xFF;
                                        break;
                                }
                            }
                            else
                            {
                                errflag = 0xFF;
                            }

                            if( USBFS_SetupReqLen > 2 )
                            {
                                USBFS_SetupReqLen = 2;
                            }

                            break;

                        default:
                            errflag = 0xFF;
                            break;
                    }
                }
                /* errflag = 0xFF means a request not support or some errors occurred, else correct */
                if( errflag == 0xff)
                {
                    /* if one request not support, return stall */
                    USBFSD->UEP0_CTRL_H = USBFS_UEP_T_TOG|USBFS_UEP_T_RES_STALL|USBFS_UEP_R_TOG|USBFS_UEP_R_RES_STALL;
                }
                else
                {
                    /* end-point 0 data Tx/Rx */
                    if( USBFS_SetupReqType & DEF_UEP_IN )
                    {
                        /* tx */
                        len = (USBFS_SetupReqLen>DevEP0SIZE) ? DevEP0SIZE : USBFS_SetupReqLen;
                        USBFS_SetupReqLen -= len;

                    }
                    else
                    {
                        len = 0;
                    }
                    USBFSD->UEP0_TX_LEN  = len;
                    USBFSD->UEP0_CTRL_H = USBFS_UEP_T_TOG|USBFS_UEP_T_RES_ACK|USBFS_UEP_R_TOG|USBFS_UEP_R_RES_ACK;

                }
                break;

            /* Sof pack processing */
            case USBFS_UIS_TOKEN_SOF:
                break;

            default :
                break;
        }
        USBFSD->INT_FG = USBFS_UIF_TRANSFER;
    }
    else if( intflag & USBFS_UIF_BUS_RST )
    {
        /* usb reset interrupt processing */
        USBFSD->DEV_ADDR = 0;
        USBFS_Device_Endp_Init( );
        USBFSD->INT_FG = USBFS_UIF_BUS_RST;
    }
    else if( intflag & USBFS_UIF_SUSPEND )
    {
        /* usb suspend interrupt processing */
        if ( USBFSD->MIS_ST & USBFS_UMS_SUSPEND )
        {
            USBFS_DevSleepStatus |= 0x02;
            if( USBFS_DevSleepStatus == 0x03 )
            {
                /* Handling usb sleep here */
            }
        }
        else
        {
            USBFS_DevSleepStatus &= ~0x02;
        }
        USBFSD->INT_FG = USBFS_UIF_SUSPEND;
    }
    else
    {
        /* other interrupts */
        USBFSD->INT_FG = intflag;
    }
}

