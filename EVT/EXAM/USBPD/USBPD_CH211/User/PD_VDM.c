/********************************** (C) COPYRIGHT  *******************************
 * File Name          : PD_VDM.c
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
#include "debug.h"
#include "PD_Prot.h"
#include "libCH32_USBPD.h"
#include "PD_VDM.h"
#include "string.h"
#include "string.h"
#include"ch32x035_usbpd.h"
st_VDM VDM_State = {0};
st_VDM_Header *rxVDM = (st_VDM_Header *)&PD_RX_BUF[1];
st_VDM_Header *txVDM = (st_VDM_Header *)&PD_TX_BUF[1];

u8   VDM_Ident_PD2[ ] =
{
	0x86, 0x1A, 0x00, 0x6C,  //ID_Header
	0xE6, 0x36, 0x00, 0x00,   //XID
	0x00, 0x00, 0x35, 0xF0,  //bcdDevice PID  F035
	0x08, 0x00, 0x00, 0x11   //AMA VDO
};

u8  VDM_Ident_PD3[ ] =
{
	0x86, 0x1A, 0x40, 0x54,  //ID_Header
	0xE6, 0x36, 0x00, 0x00,   //XID
	0x00, 0x00, 0x35, 0xF0,  //bcdDevice PID
	0x48, 0x00, 0x00, 0x01   //UFP VDO
};
u8 VDM_SVID[ 4 ] ={ 0x00, 0x00, 0x01, 0xFF  };

u8 VDM_Mode[ 4 ] = { 0x05, 0x0C, 0x00, 0x00 };

void (*VDM_REQ_Msg_Handle[][2])() =
    {
										/* DFP */			                /* UFP */
	/* enum_PD_VDM_DISC_IDENT	    */	pProt_RX_REQ_IDENT_DFP,	        pProt_RX_REQ_IDENT_UFP,
	/* enum_PD_VDM_DISC_SVID		*/	pProt_RX_REQ_SVID_DFP,	 		pProt_RX_REQ_SVID_UFP,
	/* enum_PD_VDM_DISC_MODE		*/	pProt_RX_REQ_MODE_DFP,		    pProt_RX_REQ_MODE_UFP,
	/* enum_PD_VDM_ENTER_MODE		*/	pProt_TX_NAK,		            pProt_RX_REQ_ENTER_UFP,
	/* enum_PD_VDM_EXIT_MODE		*/	pProt_TX_NAK,		            pProt_RX_REQ_EXIT_UFP,
	/* enum_PD_VDM_ATTENTION	    */	pProt_RX_ATTENTION_DFP,		    pProt_RX_ATTENTION_UFP,

	};

void (*VDM_ACK_Msg_Handle[][2])() =
    {
								            /* DFP */			                /* UFP */
	/* enum_PD_VDM_DISC_IDENT	    */	pProt_RX_ACK_IDENT_DFP,	        pProt_RX_ACK_IDENT_UFP,
	/* enum_PD_VDM_DISC_SVID		*/	pProt_RX_ACK_SVID_DFP,	 		pProt_RX_ACK_SVID_UFP,
	/* enum_PD_VDM_DISC_MODE		*/	pProt_RX_ACK_MODE_DFP,		    pProt_RX_ACK_MODE_UFP,
	/* enum_PD_VDM_ENTER_MODE		*/	pProt_RX_ACK_ENTER_DFP,		    NULL,
	/* enum_PD_VDM_EXIT_MODE		*/	pProt_RX_ACK_EXIT_DFP,		    NULL,
	/* enum_PD_VDM_ATTENTION	    */	pProt_RX_ATTENTION_DFP,		    pProt_RX_ATTENTION_UFP,
	};

/*********************************************************************
 * @fn      pProt_RX_VDM
 *
 * @brief   Receive VDM.
 *
 * @return  none
 */
void pProt_RX_VDM(void)
{
	if( VDM_State.Explicit_Contract_Established ==0)
	{
		return;     //ignore
	}

	if(rxVDM->CommandType == 0x00)
	{

		if ( rxVDM->Command > PD_VDM_Attention ) PD_TX_NotSupported();
		else
		 {
			if(rxVDM->Type)             //structured VDM
			{
				(VDM_REQ_Msg_Handle[(rxVDM->Command)-1][(PD_PHY.Header.PortDataRole)?(0):(1)])();
			}
			else                        //unstructured VDM
			{
				if( (!PD_PHY.Header.PortDataRole) && (rxVDM->SVID == 886) ) {}
				else
					{
						if((rxHeader->SpecRevision)==1)
						{
							//ignore
						}
						else
						{
							PD_TX_NotSupported();
						}
					}
		 }
	 }
	}
	else if(rxVDM->CommandType == 0x01)
	{
		(VDM_ACK_Msg_Handle[(rxVDM->Command)-1][(PD_PHY.Header.PortDataRole)?(0):(1)])();
	}
}

/*********************************************************************
 * @fn      pProt_RX_REQ_IDENT_UFP
 *
 * @brief   Receive REQ_IDENT_UFP.
 *
 * @return  none
 */
void pProt_RX_REQ_IDENT_UFP(void)
{
	memcpy( &PD_TX_BUF[ 1 ], &rxVDM->Data, 4 );
	txVDM->CommandType = 1;
	 //ver1(pd3.1) and above use ver1
	if(rxVDM->VersionMajor >1)
	{   if(rxHeader->SpecRevision==PD_Rev3)
		{
		  txVDM->VersionMajor = 1;
		}
		else
		{
		  txVDM->VersionMajor = 0;
		}
	}
	//txVDM->VersionMinor = 1;   //PD3.0=1
	if( rxVDM->VersionMajor == 1 )
   {
		memcpy( &PD_TX_BUF[ 3 ], VDM_Ident_PD3, sizeof( VDM_Ident_PD3 ) );
		PD_PHY_Header_Init(5,(sizeof( VDM_Ident_PD3 )/4)+1,PD_Data_VendorDefined);
		PD_Prot_pSet( NULL , pProt_IDLE , pProt_TX_SoftRst , NULL );
   }
    else
   {
		memcpy( &PD_TX_BUF[ 3 ], VDM_Ident_PD2, sizeof( VDM_Ident_PD2 ) );
		PD_PHY_Header_Init(5,(sizeof( VDM_Ident_PD2 )/4)+1,PD_Data_VendorDefined);
		PD_Prot_pSet( NULL , pProt_IDLE , pProt_TX_SoftRst , NULL );
   }
}

/*********************************************************************
 * @fn      pProt_RX_REQ_SVID_UFP
 *
 * @brief   Receive REQ_SVID_UFP.
 *
 * @return  none
 */
void pProt_RX_REQ_SVID_UFP(void)
{
	memcpy( &PD_TX_BUF[ 1 ], &rxVDM->Data, 4 );
	txVDM->CommandType = 1;
	//ver1(pd3.1) and above use ver1
		if(rxVDM->VersionMajor >1)
		{   if(rxHeader->SpecRevision==PD_Rev3)
			{
			  txVDM->VersionMajor = 1;
			}
			else
			{
			  txVDM->VersionMajor = 0;
			}
		}
	memcpy( &PD_TX_BUF[ 3 ], VDM_SVID, sizeof( VDM_SVID ) );
	PD_PHY_Header_Init(5,(sizeof( VDM_SVID )/4)+1,PD_Data_VendorDefined);
	PD_Prot_pSet( NULL , pProt_IDLE , pProt_TX_SoftRst , NULL );
}

/*********************************************************************
 * @fn      pProt_RX_REQ_MODE_UFP
 *
 * @brief   Receive REQ_MODE_UFP.
 *
 * @return  none
 */
void pProt_RX_REQ_MODE_UFP(void)
{
	memcpy( &PD_TX_BUF[ 1 ], &rxVDM->Data, 4 );
	txVDM->CommandType = 1;
	//ver1(pd3.1) and above use ver1
		if(rxVDM->VersionMajor >1)
		{   if(rxHeader->SpecRevision==PD_Rev3)
			{
			  txVDM->VersionMajor = 1;
			}
			else
			{
			  txVDM->VersionMajor = 0;
			}
		}
	memcpy( &PD_TX_BUF[ 3 ], VDM_Mode, sizeof( VDM_Mode ) );
	PD_PHY_Header_Init(5,(sizeof( VDM_Mode )/4)+1,PD_Data_VendorDefined);
	PD_Prot_pSet( NULL , pProt_IDLE , pProt_TX_SoftRst , NULL );
}

/*********************************************************************
 * @fn      pProt_RX_REQ_ENTER_UFP
 *
 * @brief   Receive REQ_ENTER_UFP.
 *
 * @return  none
 */
void pProt_RX_REQ_ENTER_UFP(void)
{
	VDM_State.Enter_Mode_already = 1;
	pProt_TX_ACK();

}
/*********************************************************************
 * @fn      pProt_RX_REQ_EXIT_UFP
 *
 * @brief   Receive REQ_EXIT_UFP.
 *
 * @return  none
 */
void pProt_RX_REQ_EXIT_UFP(void)
{
	if(VDM_State.Enter_Mode_already ==1 )
	{
	pProt_TX_ACK();
	VDM_State.Enter_Mode_already = 0;
	}
	else
	{
	pProt_TX_NAK();
	}
}

/*********************************************************************
 * @fn      pProt_RX_ATTENTION_UFP
 *
 * @brief   Receive ATTENTION_UFP.
 *
 * @return  none
 */
void pProt_RX_ATTENTION_UFP(void)
{

}

/*********************************************************************
 * @fn      pProt_RX_REQ_IDENT_DFP
 *
 * @brief   Receive REQ_IDENT_DFP.
 *
 * @return  none
 */
void pProt_RX_REQ_IDENT_DFP(void)
{
	pProt_TX_NAK();
}

/*********************************************************************
 * @fn      pProt_RX_REQ_IDENT_DFP
 *
 * @brief   Receive REQ_IDENT_DFP.
 *
 * @return  none
 */
void pProt_RX_REQ_SVID_DFP(void)
{
	pProt_TX_NAK();
}

/*********************************************************************
 * @fn      pProt_RX_REQ_MODE_DFP
 *
 * @brief   Receive REQ_MODE_DFP.
 *
 * @return  none
 */
void pProt_RX_REQ_MODE_DFP(void)
{
	pProt_TX_NAK();
}

/*********************************************************************
 * @fn      pProt_RX_ATTENTION_DFP
 *
 * @brief   Receive ATTENTION_DFP.
 *
 * @return  none
 */
void pProt_RX_ATTENTION_DFP(void)
{

}

/*********************************************************************
 * @fn      pProt_TX_NAK
 *
 * @brief   Send TX_NAK.
 *
 * @return  none
 */
void pProt_TX_NAK(void)
{
   memcpy(&PD_TX_BUF[1],&rxVDM->Data,4);
   txVDM->CommandType = 2;		//NAK
   //ver1(pd3.1) and above use ver1
   	if(rxVDM->VersionMajor >1)
   	{   if(rxHeader->SpecRevision==PD_Rev3)
   		{
   		  txVDM->VersionMajor = 1;
   		}
   		else
   		{
   		  txVDM->VersionMajor = 0;
   		}
   	}
   PD_PHY_Header_Init(5,1,PD_Data_VendorDefined);
   PD_Prot_pSet( NULL , pProt_IDLE , pProt_TX_SoftRst , NULL );
}

/*********************************************************************
 * @fn      pProt_TX_ACK
 *
 * @brief   Send ACK.
 *
 * @return  none
 */
void pProt_TX_ACK(void)
{
   memcpy(&PD_TX_BUF[1],&rxVDM->Data,4);
   txVDM->CommandType = 1;		//ACK
   //ver1(pd3.1) and above use ver1
   	if(rxVDM->VersionMajor >1)
   	{   if(rxHeader->SpecRevision==PD_Rev3)
   		{
   		  txVDM->VersionMajor = 1;
   		}
   		else
   		{
   		  txVDM->VersionMajor = 0;
   		}
   	}
   PD_PHY_Header_Init(5,1,PD_Data_VendorDefined);
   PD_Prot_pSet( NULL , pProt_IDLE , pProt_TX_SoftRst , NULL );
}

/*********************************************************************
 * @fn      pProt_TX_DISC_IDENT
 *
 * @brief   Send DISC_IDENT.
 *
 * @return  none
 */
void pProt_TX_DISC_IDENT(void)
{
	txVDM->SVID = 0xFF00;
	txVDM->Type = 1;
	txVDM->VersionMajor = 1;  //PD3 VER2
	txVDM->VersionMinor = 0;
	txVDM->ObjectPosition = 0;
	txVDM->CommandType = 0;
	txVDM->Reserved = 0;
	txVDM->Command = (u16)0X01;
	PD_PHY_Header_Init(5,1,PD_Data_VendorDefined);
	PD_Prot_pSet( NULL , pProt_IDLE , pProt_TX_SoftRst , NULL );
}

/*********************************************************************
 * @fn      pProt_RX_ACK_IDENT_DFP
 *
 * @brief   Receive ACK_IDENT_DFP.
 *
 * @return  none
 */
void pProt_RX_ACK_IDENT_DFP(void)
{
	txVDM->SVID = 0xFF00;
	txVDM->Type = 1;
	txVDM->VersionMajor = 1;  //PD3 VER2
	txVDM->VersionMinor = 0;
	txVDM->ObjectPosition = 0;
	txVDM->CommandType = 0;
	txVDM->Reserved = 0;
	txVDM->Command = (u16)0X02;
	PD_PHY_Header_Init(5,1,PD_Data_VendorDefined);
	PD_Prot_pSet( NULL , pProt_IDLE , pProt_TX_SoftRst , NULL );
}

/*********************************************************************
 * @fn      pProt_RX_ACK_SVID_DFP
 *
 * @brief   Receive ACK_SVID_DFP.
 *
 * @return  none
 */
void pProt_RX_ACK_SVID_DFP(void)
{
	txVDM->SVID = 0xFF00;
	txVDM->Type = 1;
	txVDM->VersionMajor = 1;  //PD3 VER2
	txVDM->VersionMinor = 0;
	txVDM->ObjectPosition = 0;
	txVDM->CommandType = 0;
	txVDM->Reserved = 0;
	txVDM->Command = (u16)0X03;
	PD_PHY_Header_Init(5,1,PD_Data_VendorDefined);
	PD_Prot_pSet( NULL , pProt_IDLE , pProt_TX_SoftRst , NULL );
}

/*********************************************************************
 * @fn      pProt_RX_ACK_MODE_DFP
 *
 * @brief   Receive ACK_MODE_DFP.
 *
 * @return  none
 */
void pProt_RX_ACK_MODE_DFP(void)
{
	txVDM->SVID = 0xFF00;
	txVDM->Type = 1;
	txVDM->VersionMajor = 1;  //PD3 VER2
	txVDM->VersionMinor = 0;
	txVDM->ObjectPosition = 0X01;
	txVDM->CommandType = 0;
	txVDM->Reserved = 0;
	txVDM->Command = (u16)0X04;
	PD_PHY_Header_Init(5,1,PD_Data_VendorDefined);
	PD_Prot_pSet( NULL , pProt_IDLE , pProt_TX_SoftRst , NULL );
}

/*********************************************************************
 * @fn      pProt_RX_ACK_ENTER_DFP
 *
 * @brief   Receive ACK_ENTER_DFP.
 *
 * @return  none
 */
void pProt_RX_ACK_ENTER_DFP(void)
{
	txVDM->SVID = 0xFF00;
	txVDM->Type = 1;
	txVDM->VersionMajor = 1;  //PD3 VER2
	txVDM->VersionMinor = 0;
	txVDM->ObjectPosition = 0;
	txVDM->CommandType = 0;
	txVDM->Reserved = 0;
	txVDM->Command = (u16)0X05;
	PD_PHY_Header_Init(5,1,PD_Data_VendorDefined);
	PD_Prot_pSet( NULL , pProt_IDLE , pProt_TX_SoftRst , NULL );
}

/*********************************************************************
 * @fn      pProt_RX_ACK_EXIT_DFP
 *
 * @brief   Receive ACK_EXIT_DFP.
 *
 * @return  none
 */
void pProt_RX_ACK_EXIT_DFP(void)
{

}

/*********************************************************************
 * @fn      pProt_RX_ACK_IDENT_UFP
 *
 * @brief   Receive ACK_IDENT_UFP.
 *
 * @return  none
 */
void pProt_RX_ACK_IDENT_UFP(void)
{
	txVDM->SVID = 0xFF00;
	txVDM->Type = 1;
	txVDM->VersionMajor = 1;  //PD3 VER2
	txVDM->VersionMinor = 0;
	txVDM->ObjectPosition = 0;
	txVDM->CommandType = 0;
	txVDM->Reserved = 0;
	txVDM->Command = (u16)0X02;
	PD_PHY_Header_Init(5,1,PD_Data_VendorDefined);
	PD_Prot_pSet( NULL , pProt_IDLE , pProt_TX_SoftRst , NULL );
}

/*********************************************************************
 * @fn      pProt_RX_ACK_SVID_UFP
 *
 * @brief   Receive ACK_SVID_UFP.
 *
 * @return  none
 */
void pProt_RX_ACK_SVID_UFP(void)
{
	txVDM->SVID = 0xFF00;
	txVDM->Type = 1;
	txVDM->VersionMajor = 1;  //PD3 VER2
	txVDM->VersionMinor = 0;
	txVDM->ObjectPosition = 0;
	txVDM->CommandType = 0;
	txVDM->Reserved = 0;
	txVDM->Command = (u16)0X03;
	PD_PHY_Header_Init(5,1,PD_Data_VendorDefined);
	PD_Prot_pSet( NULL , pProt_IDLE , pProt_TX_SoftRst , NULL );
}

/*********************************************************************
 * @fn      pProt_RX_ACK_MODE_UFP
 *
 * @brief   Receive ACK_MODE_UFP.
 *
 * @return  none
 */
void pProt_RX_ACK_MODE_UFP(void)
{
	txVDM->SVID = 0xFF00;
	txVDM->Type = 1;
	txVDM->VersionMajor = 1;  //PD3 VER2
	txVDM->VersionMinor = 0;
	txVDM->ObjectPosition = 0X01;
	txVDM->CommandType = 0;
	txVDM->Reserved = 0;
	txVDM->Command = (u16)0X04;
	PD_PHY_Header_Init(5,1,PD_Data_VendorDefined);
	PD_Prot_pSet( NULL , pProt_IDLE , pProt_TX_SoftRst , NULL );
}
