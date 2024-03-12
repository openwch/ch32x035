/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/12/26
 * Description        : Main program body.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/*
 *@Note
 *
 *PC18---SWDIO,needs to connected  a 1k pull-up resistor
 *
 */

#include "debug.h"
#include "string.h"

void PIOC_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void PIOC_Single_Wire_SingleWrite(uint8_t addr,uint32_t data);

#define  Timebase_def  1

//#define    Timebase_1X  0
#define    Timebase_2X  Timebase_def
//#define    Timebase_4X  2
//#define    Timebase_8X  3

#ifdef Timebase_1X
uint8_t  Timebase = Timebase_1X;
#elif  Timebase_4X
uint8_t  Timebase = Timebase_4X;
#elif  Timebase_8X
uint8_t  Timebase = Timebase_8X;
#else
uint8_t  Timebase = Timebase_def;
#endif

uint8_t Timebase_Flag=0;
uint8_t dbg_flag = 0;


uint16_t Send_RemainLEN=0;
uint16_t Receive_RemainLEN=0;
uint32_t  *Send_BUF_ADDR = NULL;
uint32_t  *Receive_BUF_ADDR = NULL;



uint32_t pbuf1[16] = {
    0x55aa0000, 0xaa550001,
    0x55aa55aa, 0xaa55aa55,
    0x55aa55aa, 0xaa55aa55,
    0x090A0B0C, 0x0D0E0F55,
    0x5508090A, 0x0B0C0D0E,
    0x00010203, 0x04050500,
    0x01020304, 0x05060708,
    0x090A0B0C, 0x0D0E0F55,
};
uint32_t buf[16];

const u32 PIOC_Single_Wire_R_W_blk[8]={ 0xe0000537, 0x0f450513, 0xf693414c, 0x8563ffc5, 0x411000d5, 0xa019c290, 0xc1104290, 0xc14c0591};
const u32 PIOC_Single_Wire_R_32bit[8]={ 0x7b251073, 0x7b359073, 0xe0000537, 0x0f852583, 0x2a23418c, 0x25730eb5, 0x25f37b20, 0x90027b30};
const u32 PIOC_Single_Wire_R_16bit[8]={ 0x7b251073, 0x7b359073, 0xe0000537, 0x0f852583, 0x0005d583, 0x0eb52a23, 0x7b202573, 0x7b3025f3};
const u32 PIOC_Single_Wire_R_8bit[8]={ 0x7b251073, 0x7b359073, 0xe0000537, 0x0f852583, 0x0005c583, 0x0eb52a23, 0x7b202573, 0x7b3025f3};
const u32 PIOC_Single_Wire_W_32bit[8]={ 0x7b251073, 0x7b359073, 0xe0000537, 0x0f852583, 0x0f452503, 0x2573c188, 0x25f37b20, 0x90027b30};
const u32 PIOC_Single_Wire_W_16bit[8]={ 0x7b251073, 0x7b359073, 0xe0000537, 0x0f852583, 0x0f452503, 0x00a59023, 0x7b202573, 0x7b3025f3};
const u32 PIOC_Single_Wire_W_8bit[8]={ 0x7b251073, 0x7b359073, 0xe0000537, 0x0f852583, 0x0f452503, 0x00a58023, 0x7b202573, 0x7b3025f3};

__attribute__((aligned(16)))  const unsigned char PIOC_CODE[] =
{0x00,0x00,0xF4,0x60,0xFF,0x0F,0x04,0x00,0x07,0x10,0x03,0x28,0x20,0x09,0x07,0x2B,   /* ...`.......(...+ */
 0x06,0x10,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,   /* ..0............. */
 0x30,0x00,0x0A,0x70,0x00,0x00,0x0A,0x70,0x00,0x00,0x0A,0x70,0x00,0x00,0x0A,0x70,   /* 0..p...p...p...p */
 0x00,0x00,0xFF,0x2C,0x00,0x00,0x11,0x30,0x30,0x00,0x00,0x00,0x00,0x00,0x30,0x00,   /* ...,...00.....0. */
 0x1D,0x70,0x1D,0x70,0x1D,0x70,0x1D,0x70,0x1D,0x70,0x1D,0x70,0x00,0x00,0x00,0x00,   /* .p.p.p.p.p.p.... */
 0x30,0x00,0x20,0x51,0x24,0x70,0x30,0x00,0x20,0x51,0x20,0x70,0x30,0x00,0x20,0x50,   /* 0..Q$p0..Q.p0..P */
 0x2C,0x70,0x20,0x51,0x22,0x70,0x1D,0x70,0x1D,0x70,0x00,0x00,0x00,0x00,0x00,0x00,   /* ,p.Q"p.p.p...... */
 0x00,0x00,0x30,0x00,0x0B,0x40,0x20,0x50,0x29,0x70,0x20,0x51,0x24,0x70,0x03,0x59,   /* ..0..@.P)p.Q$p.Y */
 0x2F,0x70,0x00,0x00,0x0B,0x48,0x1D,0x70,0x1D,0x70,0x1D,0x70,0x03,0x51,0x22,0x14,   /* .p...H.p.p.p.Q". */
 0x03,0x41,0x30,0x00,0x05,0x01,0x0B,0x40,0x0A,0x48,0x06,0x4D,0x20,0x50,0x29,0x70,   /* .A0....@.H.M.P)p */
 0x20,0x51,0x24,0x70,0x03,0x41,0x03,0x28,0x0A,0x40,0x17,0x00,0x06,0x45,0x0C,0x28,   /* .Q$p.A.(.@...E.( */
 0x05,0x0D,0x03,0x52,0x03,0x49,0x03,0x50,0x03,0x49,0x03,0x51,0x22,0x14,0x1D,0x70,   /* ...R.I.P.I.Q"..p */
 0x30,0x00,0x14,0x00,0x1E,0x02,0x03,0x70,0x22,0x01,0x03,0x41,0x0A,0x48,0x04,0x28,   /* 0......p"..A.H.( */
 0x21,0x10,0x20,0x52,0x03,0x49,0x3A,0x70,0x20,0x5A,0x88,0x60,0x23,0x02,0x1F,0x10,   /* !..R.I:p.Z.`#... */
 0x1F,0x56,0x03,0x49,0x3A,0x70,0x1F,0x55,0x03,0x49,0x3A,0x70,0x1F,0x54,0x03,0x49,   /* .V.I:p.U.I:p.T.I */
 0x3A,0x70,0x1F,0x53,0x03,0x49,0x3A,0x70,0x1F,0x52,0x03,0x49,0x3A,0x70,0x1F,0x51,   /* :p.S.I:p.R.I:p.Q */
 0x03,0x49,0x3A,0x70,0x1F,0x50,0x03,0x49,0x3A,0x70,0x20,0x53,0x03,0x49,0x3A,0x70,   /* .I:p.P.I:p.S.I:p */
 0x27,0x22,0x20,0x5B,0xAD,0x60,0x00,0x02,0x1F,0x10,0x1F,0x57,0x03,0x49,0x3A,0x70,   /* '".[.`.....W.I:p */
 0x1F,0x56,0x03,0x49,0x3A,0x70,0x1F,0x55,0x03,0x49,0x3A,0x70,0x1F,0x54,0x03,0x49,   /* .V.I:p.U.I:p.T.I */
 0x3A,0x70,0x1F,0x53,0x03,0x49,0x3A,0x70,0x1F,0x52,0x03,0x49,0x3A,0x70,0x1F,0x51,   /* :p.S.I:p.R.I:p.Q */
 0x03,0x49,0x3A,0x70,0x1F,0x50,0x03,0x49,0x3A,0x70,0x04,0x15,0x21,0x15,0x03,0x5A,   /* .I:p.P.I:p..!..Z */
 0x8B,0x60,0x22,0x50,0x03,0x49,0x3A,0x70,0xD0,0x60,0x1F,0x01,0x4A,0x70,0x03,0x51,   /* .`"P.I:p.`..Jp.Q */
 0x1F,0x4F,0x4A,0x70,0x03,0x51,0x1F,0x4E,0x4A,0x70,0x03,0x51,0x1F,0x4D,0x4A,0x70,   /* .OJp.Q.NJp.Q.MJp */
 0x03,0x51,0x1F,0x4C,0x4A,0x70,0x03,0x51,0x1F,0x4B,0x4A,0x70,0x03,0x51,0x1F,0x4A,   /* .Q.LJp.Q.KJp.Q.J */
 0x4A,0x70,0x03,0x51,0x1F,0x49,0x4A,0x70,0x03,0x51,0x1F,0x48,0x1F,0x02,0x00,0x10,   /* Jp.Q.IJp.Q.H.... */
 0x04,0x15,0x03,0x41,0x21,0x15,0x03,0x5A,0xAD,0x60,0x4A,0x70,0x22,0x50,0x20,0x4C,   /* ...A!..Z.`Jp"P.L */
 0x0A,0x48,0x20,0x56,0x1C,0x4F,0x20,0x4D,0x02,0x28,0x20,0x50,0x01,0x2C,0x20,0x51,   /* .H.V.O.M.(.P.,.Q */
 0x06,0x2C,0x11,0x70,0x61,0x60,0x0B,0x48,0x0A,0x48,0xFF,0x28,0x11,0x70,0x0B,0x40,   /* .,.pa`.H.H.(.p.@ */
 0xFF,0x28,0x11,0x70,0xFF,0x28,0x11,0x70,0xFF,0x28,0x11,0x70,0xFF,0x28,0x11,0x70,   /* .(.p.(.p.(.p.(.p */
 0xFF,0x28,0x11,0x70,0xFF,0x28,0x11,0x70,0xFF,0x28,0x11,0x70,0xCC,0x28,0x11,0x70,   /* .(.p.(.p.(.p.(.p */
 0x0B,0x48,0x01,0x28,0x11,0x70,0x30,0x00,0x00,0x00,0x00,0x00,0xDB,0x70,0x61,0x60,   /* .H.(.p0......pa` */
 0xF4,0x60};    /* .` */

/*********************************************************************
 * @fn      PIOC_IRQHandler
 *
 * @brief   This function handles PIOC exception.
 *
 * @return  none
 */
void PIOC_IRQHandler( void )
{
    if((R8_SYS_CFG&RB_INT_REQ)!=RESET)
    {
        if((R8_DATA_REG0&0x08)!=RESET)
        {
            Send_BUF_ADDR++;
            Send_RemainLEN--;
            if(Send_RemainLEN)
            {
                R32_DATA_REG4_7 = *Send_BUF_ADDR;
                R8_DATA_REG0 &= ~(0x04);
                R8_CTRL_WR = 0X33;
            }
            else
                R8_DATA_REG0 &= ~(0x40);
        }
        else
        {
            *Receive_BUF_ADDR = R32_DATA_REG4_7;
            Receive_BUF_ADDR++;
            Receive_RemainLEN--;
            R8_DATA_REG0 &= ~(0x04);
            if(Receive_RemainLEN)
                R8_CTRL_WR = 0X33;
            else
                R8_DATA_REG0 &= ~(0x40);
        }
        R8_CTRL_RD=11;
    }
}
/*********************************************************************
 * @fn      GPIO_Toggle_INIT
 *
 * @brief   Initializes GPIOA.0
 *
 * @return  none
 */
void GPIO_Toggle_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_SetBits(GPIOA, GPIO_Pin_0);
}

/*********************************************************************
 * @fn      PIOC_INIT
 *
 * @brief   Initializes PIOC
 *
 * @return  none
 */
void PIOC_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_18;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    NVIC_EnableIRQ( PIOC_IRQn );                                        //enable PIOC interrupt
    NVIC_SetPriority(PIOC_IRQn,0xf0);

    memcpy((uint8_t *)(PIOC_SRAM_BASE),PIOC_CODE,sizeof(PIOC_CODE));    // load code for PIOC
    R8_SYS_CFG |= RB_MST_RESET;                                         // reset PIOC
    R8_SYS_CFG = RB_MST_IO_EN0;                                          // enable IO0
    R8_SYS_CFG |= RB_MST_CLK_GATE;                                      // open PIOC clock
}

/*********************************************************************
 * @fn      PIOC_Single_Wire_INIT
 *
 * @brief   Initializes PIOC_Single_Wire
 *
 * @return  none
 */
void PIOC_Single_Wire_INIT(void)
{
    if(Timebase_Flag==RESET)
        R8_DATA_REG0 |= Timebase_def;
    PIOC_Single_Wire_SingleWrite(0x7E,0X5AA50400);
    PIOC_Single_Wire_SingleWrite(0x7D,0X5AA50400);
    if((Timebase != Timebase_def)||(Timebase_Flag!=RESET))
    {
        PIOC_Single_Wire_SingleWrite(0x7E,0X5AA50000 | Timebase);
        PIOC_Single_Wire_SingleWrite(0x7D,0X5AA50003);
    }
    Timebase_Flag=1;
    R8_DATA_REG0 &= ~(0x03);
    R8_DATA_REG0 |= Timebase;
}

/*********************************************************************
 * @fn      PIOC_Single_Wire_SingleWrite
 *
 * @brief   single wire single send.
 *
 * @param   addr - address.
 *          data - 32-bit data.
 *
 * @return  none
 */
void PIOC_Single_Wire_SingleWrite(uint8_t addr,uint32_t data)
{
    while((R8_DATA_REG0&0x40)!=RESET);
    R8_DATA_REG3 = addr;
    R32_DATA_REG4_7 = data;
    R8_DATA_REG0 &= ~(0x20);
    R8_DATA_REG0 |= 0x0C;
    R8_CTRL_WR = 0X33;
    while((R8_DATA_REG0&0x20)==RESET);          //Waiting for sending to complete
}

/*********************************************************************
 * @fn      PIOC_Single_Wire_SingleRead
 *
 * @brief   single wire single receive.
 *
 * @param   addr - address.
 *          data - 32-bit data.
 *
 * @return  even parity check
 *          1 - Parity error
 */
uint8_t PIOC_Single_Wire_SingleRead(uint8_t addr,uint32_t* data)
{
    while((R8_DATA_REG0&0x40)!=RESET);
    R8_DATA_REG3 = addr;
    R8_DATA_REG0 &= ~(0x2C);
    R8_DATA_REG0 |= 0x04;
    R8_CTRL_WR = 0X33;
    while((R8_DATA_REG0&0x20)==RESET);          //Waiting for reception to complete
    *data = R32_DATA_REG4_7;
    if((R8_DATA_REG0&0x10)!=RESET)
        return 1;
    return 0;
}

/*********************************************************************
 * @fn      PIOC_Single_Wire_ContinuityWrite
 *
 * @brief   single wire continuous send.
 *
 * @param   addr - address.
 *          data - 32-bit data.
 *          len  - sending data length.
 *
 * @return  none
 */
void PIOC_Single_Wire_ContinuityWrite(uint8_t addr,uint32_t* data,uint16_t len)
{
    while((R8_DATA_REG0&0x40)!=RESET);
    R8_DATA_REG3 = addr;
    R32_DATA_REG4_7 = *data;
    Send_BUF_ADDR = data;
    Send_RemainLEN = len;
    R8_DATA_REG0 |= 0x4C;
    R8_CTRL_WR = 0X33;
}

/*********************************************************************
 * @fn      PIOC_Single_Wire_ContinuityRead
 *
 * @brief   single wire continuous receive.
 *
 * @param   addr - address.
 *          data - 32-bit data.
 *          len  - sending data length.
 *
 * @return  none
 */
void PIOC_Single_Wire_ContinuityRead(uint8_t addr,uint32_t* data,uint16_t len)
{
    while((R8_DATA_REG0&0x40)!=RESET);
    R8_DATA_REG3 = addr;
    Receive_BUF_ADDR = data;
    Receive_RemainLEN = len;
    R8_DATA_REG0 &= ~(0x0C);
    R8_DATA_REG0 |= 0x44;
    R8_CTRL_WR = 0X33;
}

/*********************************************************************
 * @fn      PIOC_Single_Wire_ABSTRTS
 *
 * @brief   Waiting for abstract command execution to complete.
 *
 * @return  0 - success
 *          1 - fail
 */
uint8_t PIOC_Single_Wire_ABSTRTS(void)
{
    uint32_t tmp=0;
    uint8_t  i=0;
    while(1)
    {
        PIOC_Single_Wire_SingleRead(0X16,&tmp);
        if((tmp&(1<<12)) == 0)  break;
        Delay_Ms(5);
        i++;
        if(i>10)  return 1;
    }
    return 0;
}

/*********************************************************************
 * @fn      PIOC_Single_Wire_Pause
 *
 * @brief   Pause the slave kernel
 *
 * @return  0 - success
 *          1 - fail
 */
uint8_t PIOC_Single_Wire_Pause(void)
{
    uint32_t tmp=0;
    uint8_t  i=0;

    PIOC_Single_Wire_SingleWrite(0X10,0X80000001);
    PIOC_Single_Wire_SingleWrite(0X10,0X80000001);
    PIOC_Single_Wire_SingleRead(0X11,&tmp);
    while(1)
    {
        if(((tmp&(3<<8))==(3<<8))&&((tmp&(0x000F0FFF)) != (0x000F0FFF)))
            break;
        Delay_Ms(5);
        PIOC_Single_Wire_SingleWrite(0X10,0X80000001);
        PIOC_Single_Wire_SingleRead(0X11,&tmp);
        i++;
        if(i>10)  return 1;
    }
    PIOC_Single_Wire_SingleWrite(0X10,0X00000001);
    return 0;
}

/*********************************************************************
 * @fn      PIOC_Single_Wire_Exit_Pause
 *
 * @brief   Exit Pause the slave kernel
 *
 * @return  0 - success
 *          1 - fail
 */
uint8_t PIOC_Single_Wire_Exit_Pause(void)
{
    uint32_t tmp=0;
    uint8_t  i=0;

    PIOC_Single_Wire_ABSTRTS();
    PIOC_Single_Wire_SingleWrite(0X10,0X40000001);
    PIOC_Single_Wire_SingleRead(0X11,&tmp);
    while(1)
    {
        if((((tmp&(3<<16))==(3<<16))||((tmp&(3<<10))==(3<<10)))&&((tmp&(0x000F0FFF)) != (0x000F0FFF)))
            break;
        Delay_Ms(5);
        PIOC_Single_Wire_SingleWrite(0X10,0X40000001);
        PIOC_Single_Wire_SingleRead(0X11,&tmp);
        i++;
        if(i>10)  return 1;
    }
    return 0;
}

/*********************************************************************
 * @fn      PIOC_Single_Wire_R_W_blk_set
 *
 * @brief   Write pending instructions to progbuf.
 *          Batch read and write only supports 32bit,
 *          with low order 0 for read addresses and 1 for write addresses
 *
 * @return  none
 */
void PIOC_Single_Wire_R_W_blk_set( void )
{
    u8 i;
    if(dbg_flag!=(1<<1))
    {
        for(i=0;i<(u8)(sizeof(PIOC_Single_Wire_R_W_blk)/4);i++)
        {
            PIOC_Single_Wire_SingleWrite(0X20+i,PIOC_Single_Wire_R_W_blk[i]);
        }
        dbg_flag = (1<<1);
    }
}

/*********************************************************************
 * @fn      PIOC_Single_Wire_R_32bit_set
 *
 * @brief   Write pending instructions to progbuf.
 *          Single read 32-bit length data instruct.
 *
 * @return  none
 */
void PIOC_Single_Wire_R_32bit_set( void )
{
    u8 i;
    if(dbg_flag!=(1<<2))
    {
        for(i=0;i<(u8)(sizeof(PIOC_Single_Wire_R_32bit)/4);i++)
        {
            PIOC_Single_Wire_SingleWrite(0X20+i,PIOC_Single_Wire_R_32bit[i]);
        }
        dbg_flag = (1<<2);
    }
}

/*********************************************************************
 * @fn      PIOC_Single_Wire_W_32bit_set
 *
 * @brief   Write pending instructions to progbuf.
 *          Single write 32-bit length data instruct.
 *
 * @return  none
 */
void PIOC_Single_Wire_W_32bit_set( void )
{
    u8 i;
    if(dbg_flag!=(1<<3))
    {
        for(i=0;i<(u8)(sizeof(PIOC_Single_Wire_W_32bit)/4);i++)
        {
            PIOC_Single_Wire_SingleWrite(0X20+i,PIOC_Single_Wire_W_32bit[i]);
        }
        dbg_flag = (1<<3);
    }
}

/*********************************************************************
 * @fn      PIOC_Single_Wire_R_8bit_set
 *
 * @brief   Write pending instructions to progbuf.
 *          Single read 8-bit length data instruct.
 *
 * @return  none
 */
void PIOC_Single_Wire_R_8bit_set( void )
{
    u8 i;
    if(dbg_flag!=(1<<4))
    {
        for(i=0;i<(u8)(sizeof(PIOC_Single_Wire_R_8bit)/4);i++)
        {
            PIOC_Single_Wire_SingleWrite(0X20+i,PIOC_Single_Wire_R_8bit[i]);
        }
        dbg_flag = (1<<4);
    }
}

/*********************************************************************
 * @fn      PIOC_Single_Wire_W_8bit_set
 *
 * @brief   Write pending instructions to progbuf.
 *          Single write 8-bit length data instruct.
 *
 * @return  none
 */
void PIOC_Single_Wire_W_8bit_set( void )
{
    u8 i;
    if(dbg_flag!=(1<<5))
    {
        for(i=0;i<(u8)(sizeof(PIOC_Single_Wire_W_8bit)/4);i++)
        {
            PIOC_Single_Wire_SingleWrite(0X20+i,PIOC_Single_Wire_W_8bit[i]);
        }
        dbg_flag = (1<<5);
    }
}

/*********************************************************************
 * @fn      PIOC_Single_Wire_R_16bit_set
 *
 * @brief   Write pending instructions to progbuf.
 *          Single read 16-bit length data instruct.
 *
 * @return  none
 */
void PIOC_Single_Wire_R_16bit_set( void )
{
    u8 i;
    if(dbg_flag!=(1<<6))
    {
        for(i=0;i<(u8)(sizeof(PIOC_Single_Wire_R_16bit)/4);i++)
        {
            PIOC_Single_Wire_SingleWrite(0X20+i,PIOC_Single_Wire_R_16bit[i]);
        }
        dbg_flag = (1<<6);
    }
}

/*********************************************************************
 * @fn      PIOC_Single_Wire_W_16bit_set
 *
 * @brief   Write pending instructions to progbuf.
 *          Single write 16-bit length data instruct.
 *
 * @return  none
 */
void PIOC_Single_Wire_W_16bit_set( void )
{
    u8 i;
    if(dbg_flag!=(1<<7))
    {
        for(i=0;i<(u8)(sizeof(PIOC_Single_Wire_W_16bit)/4);i++)
        {
            PIOC_Single_Wire_SingleWrite(0X20+i,PIOC_Single_Wire_W_16bit[i]);
        }
        dbg_flag = (1<<7);
    }
}

/*********************************************************************
 * @fn      PIOC_Single_Wire_RAM_SingleWrite
 *
 * @brief   Write 32-bit data to RAM.
 *
 * @param   addr - RAM address.
 *          data - 32-bit data.
 *          datasize - 0-8bit 1-16bit 2-32bit
 *
 * @return  none
 */
void PIOC_Single_Wire_RAM_SingleWrite(uint32_t addr,uint32_t data,uint8_t datasize)
{
    PIOC_Single_Wire_SingleWrite(0X05,addr);
    PIOC_Single_Wire_SingleWrite(0X04,data);
    //Write pending instructions to progbuf
    if(datasize==2) PIOC_Single_Wire_W_32bit_set();
    else if(datasize==1) PIOC_Single_Wire_W_16bit_set();
    else if(datasize==0) PIOC_Single_Wire_W_8bit_set();
    //execute prosbuf
    PIOC_Single_Wire_SingleWrite(0X17,0X00040000);
}

/*********************************************************************
 * @fn      PIOC_Single_Wire_RAM_ContinuityWrite
 *
 * @brief   Batch write 32-bit data to RAM.
 *
 * @param   addr - RAM address.
 *          data - 32-bit data.
 *          len  - write data length.
 *
 * @return  none
 */
void PIOC_Single_Wire_RAM_ContinuityWrite(uint32_t addr,uint32_t* data,uint16_t len)
{
    PIOC_Single_Wire_SingleWrite(0X05,addr+1);
    PIOC_Single_Wire_SingleWrite(0X04,*data);
    //Write pending instructions to progbuf
    PIOC_Single_Wire_R_W_blk_set();
    //execute prosbuf
    PIOC_Single_Wire_SingleWrite(0X17,0X00040000);
    //Write DEG_RGG_ABSAUTO
    PIOC_Single_Wire_SingleWrite(0X18,0X00000001);
    //Batch write data0
    PIOC_Single_Wire_ContinuityWrite(0x04,data+1,len-1);
    //Write DEG_RGG_ABSAUTO
    PIOC_Single_Wire_SingleWrite(0X18,0X00000000);
}

/*********************************************************************
 * @fn      PIOC_Single_Wire_RAM_SingleRead
 *
 * @brief   Reading 32-bit data from RAM.
 *
 * @param   addr - RAM address.
 *          data - 32-bit data.
 *          datasize - 0-8bit 1-16bit 2-32bit
 *
 * @return  none
 */
void PIOC_Single_Wire_RAM_SingleRead(uint32_t addr,uint32_t* data,uint8_t datasize)
{
    //Write DEG_RGG_DATA1
    PIOC_Single_Wire_SingleWrite(0X05,addr);
    //Write pending instructions to progbuf
    if(datasize==2) PIOC_Single_Wire_R_32bit_set();
    else if(datasize==1) PIOC_Single_Wire_R_16bit_set();
    else if(datasize==0) PIOC_Single_Wire_R_8bit_set();
    //execute prosbuf
    PIOC_Single_Wire_SingleWrite(0X17,0X00040000);
    //Read data0
    PIOC_Single_Wire_SingleRead(0X04,data);
}

/*********************************************************************
 * @fn      PIOC_Single_Wire_RAM_ContinuityRead
 *
 * @brief   Batch read 32-bit data from RAM.
 *
 * @param   addr - RAM address.
 *          data - 32-bit data.
 *          len  - read data length.
 *
 * @return  none
 */
void PIOC_Single_Wire_RAM_ContinuityRead(uint32_t addr,uint32_t* data,uint16_t len)
{
    //Write DEG_RGG_ABSAUTO
    PIOC_Single_Wire_SingleWrite(0X18,0X00000001);
    //Write DEG_RGG_DATA1
    PIOC_Single_Wire_SingleWrite(0X05,addr);
    //Write pending instructions to progbuf
    PIOC_Single_Wire_R_W_blk_set();
    //execute prosbuf
    PIOC_Single_Wire_SingleWrite(0X17,0X00040000);
    //Batch read data0
    PIOC_Single_Wire_ContinuityRead(0x04,data,len-1);
    //Write DEG_RGG_ABSAUTO
    PIOC_Single_Wire_SingleWrite(0X18,0X00000000);
    //Read data0
    PIOC_Single_Wire_SingleRead(0X04,&data[len-1]);
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    uint16_t i=0;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );
    printf("GPIO Toggle TEST\r\n");

    GPIO_Toggle_INIT();
    PIOC_INIT();
    PIOC_Single_Wire_INIT();
    PIOC_Single_Wire_Pause();
    PIOC_Single_Wire_RAM_ContinuityWrite(0x20000400,pbuf1,16);
    PIOC_Single_Wire_RAM_ContinuityRead(0x20000400,buf,16);

    for(i=0;i<16;i++)
    {
        printf("%08x\r\n",buf[i]);
    }

    PIOC_Single_Wire_Exit_Pause();


    while(1)
    {
        Delay_Ms(500);
        GPIO_WriteBit(GPIOA, GPIO_Pin_0, (i == 0) ? (i = Bit_SET) : (i = Bit_RESET));
    }
}
