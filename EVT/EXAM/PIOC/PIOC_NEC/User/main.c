/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/10/31
 * Description        : Main program body.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
/*
 *@Note
 * NEC Infrared remote control receiving example:
 * Receive pin PC18\PC7 or PC19.the Fsys requires 48Mhz.
 */

#include "debug.h"
#include "string.h"

void PIOC_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

__IO  uint8_t  stat = 0;                // Receive status
__IO  uint8_t  rep_cnt = 0;             // Received duplicate code count
__IO  uint8_t  rec_flag = 0;            // Received a frame of data flag
__IO  uint32_t RecData = 0;             // Received data

#define     VRIF_ADD_ERR    0x1FF       // error code for Verify address error
#define     VRIF_VAL_ERR    0X2FF       // error code for Verify value error

__attribute__((aligned(16)))  const unsigned char PIOC_CODE[] =
{0x00,0x00,0xB7,0x60,0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,   /* ...`............ */
 0x00,0x00,0x30,0x00,0x32,0x28,0x0E,0x60,0x0A,0x28,0x0E,0x60,0x03,0x70,0x00,0x00,   /* ..0.2(.`.(.`.p.. */
 0x03,0x70,0x00,0x00,0x03,0x70,0x00,0x00,0x03,0x70,0x00,0x00,0xFF,0x2C,0x00,0x00,   /* .p...p...p...,.. */
 0x0E,0x30,0x30,0x00,0x14,0x00,0x1E,0x02,0x20,0x50,0x6C,0x60,0x25,0x60,0x40,0x28,   /* .00......Pl`%`@( */
 0x1D,0x10,0x1C,0x4F,0x20,0x50,0x6C,0x60,0x00,0x00,0x0B,0x48,0x0A,0x40,0x16,0x00,   /* ...O.Pl`...H.@.. */
 0x21,0x01,0x04,0x28,0x23,0x10,0x08,0x28,0x22,0x10,0x2B,0x22,0x17,0x00,0x0A,0x70,   /* !..(#..(".+"...p */
 0x21,0x14,0x0B,0x52,0x2F,0x60,0x21,0x02,0xAE,0x2F,0x20,0x38,0xBC,0x2F,0x20,0x3C,   /* !..R.`!....8...< */
 0x21,0x01,0x16,0x00,0x0A,0x70,0x21,0x14,0x0B,0x5A,0x3A,0x60,0x21,0x02,0x26,0x2F,   /* !....p!..Z:`!.&. */
 0x20,0x38,0x34,0x2F,0x1F,0x38,0x52,0x2F,0x20,0x38,0x60,0x2F,0x20,0x3C,0x48,0x60,   /* .84..8R..8`..<H` */
 0x17,0x00,0x21,0x01,0x20,0x41,0x16,0x00,0x0C,0x70,0x21,0x14,0x0B,0x5A,0x4C,0x60,   /* ..!..A...p!..ZL` */
 0x21,0x02,0x1E,0x2F,0x20,0x38,0x50,0x2F,0x5A,0x38,0x82,0x2F,0x20,0x38,0xBE,0x2F,   /* !....8P.Z8...8.. */
 0x20,0x3C,0x20,0x49,0x24,0x1E,0x20,0x51,0x24,0x14,0x22,0x15,0x03,0x5A,0x48,0x60,   /* .<.I$..Q$."..ZH` */
 0x08,0x28,0x22,0x10,0x24,0x02,0x24,0x01,0x00,0x10,0x04,0x15,0x23,0x15,0x03,0x5A,   /* .(".$.$.....#..Z */
 0x48,0x60,0x80,0x28,0x20,0x60,0x00,0x00,0x0B,0x41,0x0A,0x41,0x15,0x00,0x21,0x01,   /* H`.(.`...A.A..!. */
 0x04,0x28,0x23,0x10,0x08,0x28,0x22,0x10,0x2B,0x22,0x0B,0x49,0x15,0x00,0x0A,0x70,   /* .(#..(".+".I...p */
 0x21,0x14,0x0B,0x53,0x77,0x60,0x21,0x02,0xAE,0x2F,0x20,0x38,0xBC,0x2F,0x20,0x3C,   /* !..Sw`!....8...< */
 0x21,0x01,0x0B,0x41,0x15,0x00,0x0A,0x70,0x21,0x14,0x0B,0x53,0x83,0x60,0x21,0x02,   /* !..A...p!..S.`!. */
 0x26,0x2F,0x20,0x38,0x34,0x2F,0x1F,0x38,0x52,0x2F,0x20,0x38,0x60,0x2F,0x20,0x3C,   /* &..84..8R..8`..< */
 0x91,0x60,0x0B,0x49,0x15,0x00,0x21,0x01,0x20,0x41,0x0B,0x41,0x15,0x00,0x0C,0x70,   /* .`.I..!..A.A...p */
 0x21,0x14,0x0B,0x53,0x97,0x60,0x21,0x02,0x1E,0x2F,0x20,0x38,0x50,0x2F,0xA5,0x38,   /* !..S.`!....8P..8 */
 0x82,0x2F,0x20,0x38,0xBE,0x2F,0x20,0x3C,0x20,0x49,0x24,0x1E,0x20,0x51,0x24,0x14,   /* ...8...<.I$..Q$. */
 0x22,0x15,0x03,0x5A,0x91,0x60,0x08,0x28,0x22,0x10,0x24,0x02,0x24,0x01,0x00,0x10,   /* "..Z.`.(".$.$... */
 0x04,0x15,0x23,0x15,0x03,0x5A,0x91,0x60,0x80,0x28,0x20,0x60,0x00,0x00,0x00,0x00,   /* ..#..Z.`.(.`.... */
 0x00,0x00,0x1A,0x60,0xB7,0x60};    /* ...`.` */

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
        stat = PIOC->D8_CTRL_RD;        // Auto remove interrupt request after reading
        switch (stat) {
            case 0x80:                  // Received a frame of data
                {
                    RecData = PIOC->D32_DATA_REG8_11;
                    rep_cnt = 0;
                    rec_flag = 1;
                    break;
                }
            case 0x40:                  // Received duplicate code
                {
                    rep_cnt++;
                    if(rep_cnt > 10)
                    {
                        RecData = PIOC->D32_DATA_REG8_11;
                        rep_cnt = 0;
                        rec_flag = 1;
                    }
                    break;
                }
            default:                    // Error code
                printf("error.\r\n");
                break;
        }
        stat = 0;
        R8_CTRL_RD=11;                  // False write any value to R8_CTRL_RD, clear interrupt flag
    }
}

/*********************************************************************
 * @fn      PIOC_INIT
 *
 * @brief   Initializes PIOC.
 *
 * @return  none
 */
void PIOC_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_IO2W, ENABLE);

#if 1 //PC18 PC19
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_18|GPIO_Pin_19;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

#else //PC7
    GPIO_PinRemapConfig(GPIO_Remap_PIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
#endif

    NVIC_EnableIRQ( PIOC_IRQn );                                        // Enable PIOC interrupt
    NVIC_SetPriority(PIOC_IRQn,0xf0);

    memcpy((uint8_t *)(PIOC_SRAM_BASE),PIOC_CODE,sizeof(PIOC_CODE));    // Load code for PIOC
    R8_SYS_CFG |= RB_MST_RESET;                                         // Reset PIOC
    R8_SYS_CFG = RB_MST_IO_EN0|RB_MST_IO_EN1;                           // Enable IO0&IO1
    R8_SYS_CFG |= RB_MST_CLK_GATE;                                      // Open PIOC clock
}

/*********************************************************************
 * @fn      PIOC_REMOTE_INIT
 *
 * @brief   Initializes PIOC NEC remote.
 *
 * @param   mod - 0:receive pin PC18 or PC7
 *                1:receive pin PC19
 *
 * @return  none
 */
void PIOC_REMOTE_INIT(uint8_t mod)
{
    if(mod)
        PIOC->D8_DATA_REG0 = 1;
    else
        PIOC->D8_DATA_REG0 = 0;

    R8_CTRL_WR = 0X33;              // To R8_CTRL_WR write any value,then start
}

/*********************************************************************
 * @fn      PIOC_REMOTE_SCAN
 *
 * @brief   Obtain remote control data.
 *          RecData:
 *          bit[31:24] -- address
 *          bit[23:16] -- address inverse code
 *          bit[15:8]  -- command
 *          bit[7:0]   -- command inverse code
 *
 * @return  value
 */
uint16_t PIOC_REMOTE_SCAN(void)
{
    uint8_t val = 0;
    uint8_t add,add_invr,cmd,cmd_invr;
    if(rec_flag)
    {
        rec_flag = 0;
        add = RecData >> 24;
        add_invr = (RecData >> 16) & 0xFF;
        if(add == (u8)~add_invr){               // Verify address
            cmd = (RecData>>8) & 0xFF;
            cmd_invr = RecData & 0xFF;
            if(cmd == (u8)~cmd_invr)            // Verify command
                val = cmd;
            else
                return( VRIF_VAL_ERR );
        }else{
            return( VRIF_ADD_ERR );
        }
        return val;
    }
    return 0;
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
    uint16_t key_value = 0;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );
    printf( "PIOC Infrared remote control test.");
    PIOC_INIT();
    PIOC_REMOTE_INIT(0);

    while(1)
    {
        key_value = PIOC_REMOTE_SCAN();

        if(key_value)
        {
            if(key_value == VRIF_ADD_ERR){
                printf("Verify remote control address err.\r\n");
            }
            else if(key_value == VRIF_VAL_ERR){
                printf("Verify remote control value err.\r\n");
            }
            else
                printf("key value :%d\r\n",key_value);
        }
    }
}
