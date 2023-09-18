// 1-wire example: 1W-RGB, 1W-DS1820

//__align(4)	

#include    "RGB1W.h"
const unsigned char PIOC_1W_CODE[] =
#include    "RGB1W_inc.h"

#include	"RGB1W.h"
#include	<string.h>

__IO	uint8_t		stat;


//u8 RGBpbuf1[0] = { 0xFF, 0x00, 0x00,            //G R B
//                     0x00, 0xFF, 0x00,
//                     0x00, 0x00, 0xFF,
//                     0xFF, 0x00, 0x00,
//                     0x00, 0xFF, 0x00,
//                     0x00, 0x00, 0xFF,
//                     0xFF, 0x00, 0x00,
//                     0x00, 0xFF, 0x00,
//                     0x00, 0x00, 0xFF,
//                     0xFF, 0x00, 0x00,

//};

//
 uint8_t RGBpbuf1[  RGB_Total_Number ][ 3 ] = {};        //驱动RGB的灯光数据


// uint8_t RGBpbuf[ ][ 3 ] = {
//         0x30, 0x00, 0x00,            //G R B
//         0x24, 0x0F, 0x00,
//         0x18, 0x18, 0x00,
//         0x0F, 0x24, 0x00,
//         0x00, 0x30, 0x00,
//         0x00, 0x24, 0x0F,
//         0x00, 0x18, 0x18,
//         0x00, 0x0F, 0x24,
//         0x00, 0x00, 0x30,
//         0x0F, 0x00, 0x24,
//         0x18, 0x00, 0x18,
//         0x24, 0x00, 0x0F,            // 0-11   RGB_State: 12
//  };

//  uint8_t RGBpbuf[ ][ 3 ] = {
//           0x30, 0x00, 0x00,            //G R B
//           0x28, 0x00, 0x08,
//           0x20, 0x00, 0x10,
//           0x18, 0x00, 0x18,
//           0x10, 0x00, 0x20,
//           0x08, 0x00, 0x28,
//           0x00, 0x00, 0x30,
//           0x08, 0x00, 0x28,
//           0x10, 0x00, 0x20,
//           0x18, 0x00, 0x18,
//           0x20, 0x00, 0x10,
//           0x28, 0x00, 0x08,            // 0-11   RGB_State: 12
//
//    };




void PIOC_IRQHandler( void ) {
//	uint8_t	stat;
	stat = PIOC->D8_CTRL_RD;
	printf("ePIOC_IRQHandler \r\n");
	//auto remove interrupt request after reading
//	if ( stat == RGB1W_ERR_OK ) printf("1-wire finished\r\n");
//	else printf("1-wire error %02x\r\n", stat);
//	temper = PIOC->D16_DATA_REG0_1;//for DS1820 only
}

void RGB1W_Init ( void ) {
	// PIOC->D8_SYS_CFG = 0;//halt
	PIOC->D8_SYS_CFG = RB_MST_RESET | RB_MST_IO_EN0;	// reset PIOC & enable IO0

	memcpy( (uint8_t *)(PIOC_SRAM_BASE), PIOC_1W_CODE, sizeof( PIOC_1W_CODE ) );	// load code for PIOC

#if defined SYSCLK_FREQ_24MHz_HSI
	 *(uint32_t)(PIOC_SRAM_BASE+PIOC_FREQ_CFG) = 0x00300030;	// set if Fsys=24MHz, default Fsys=48MHz
#else	// default is 48MHz
#endif

// initial pin
//	AFIO->PCFR1 |= AFIO_PCFR1_PIOC_REMAP;//select PC7 as 1-wire pin, default is PC18
//	AFIO->PCFR1 |= AFIO_PCFR1_SWJ_CFG_2;//disable debug pin if use PC18 as 1-wire pin
//	GPIOC->CFGXR = GPIOC->CFGXR & ~ ( GPIO_CFGXR_CNF18 | GPIO_CFGXR_MODE18 ) | GPIO_CFGXR_MODE18_1 | GPIO_CFGXR_CNF18_1;//AFIO @PC18
	{
		GPIO_InitTypeDef GPIO_InitStructure = {0};
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOC, ENABLE);
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_IO2W, ENABLE);

#if 1  //PC18
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
//    GPIO_SetBits(GPIOC, GPIO_Pin_18);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_18;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

#else  //PC7

    GPIO_PinRemapConfig(GPIO_Remap_PIOC, ENABLE);
//    GPIO_SetBits(GPIOC, GPIO_Pin_7);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
#endif

	}

//    NVIC_EnableIRQ(PIOC_IRQn);//enable interrupt
//    NVIC_SetPriority(PIOC_IRQn,0xf0);
}

void RGB1W_SendSFR( uint16_t total_bytes, uint8_t *p_source_addr ) {//SFR mode for 1~32 bytes data
// p_source_addr: point source data buffer start address, set NULL if copy into PIOC buffer already
	if ( total_bytes > RGB1W_SFR_SIZE ) return;
	PIOC->D8_SYS_CFG =  RB_MST_IO_EN0;//clear&halt PIOC
	PIOC->D8_SYS_CFG = RB_MST_CLK_GATE | RB_MST_IO_EN0;//run PIOC before write SFR
	if ( p_source_addr ) memcpy( RGB1W_SFR_ADDR, p_source_addr, total_bytes );//copy source data to RGB1W SFR, @PIOC run
	RGB1W_COMMAND = (uint8_t)total_bytes;// PIOC start send   方式3驱动RGB
}

void RGB1W_SendRAM( uint16_t total_bytes, uint8_t *p_source_addr ) {//RAM mode for 1~3072 bytes data
// p_source_addr: point source data buffer start address, set NULL if copy into PIOC buffer already
	if ( total_bytes > RGB1W_RAM_SIZE ) return;
	//PIOC->D8_SYS_CFG = RB_MST_RESET | RB_MST_IO_EN0;//clear&halt PIOC
	PIOC->D8_SYS_CFG =  RB_MST_IO_EN0;//clear&halt PIOC
	if ( p_source_addr ) memcpy( RGB1W_RAM_ADDR, p_source_addr, total_bytes );//copy source data to PIOC SRAM, @PIOC halt
	PIOC->D8_SYS_CFG = RB_MST_CLK_GATE | RB_MST_IO_EN0;//run PIOC after load data in SRAM
	PIOC->D16_DATA_REG0_1 = total_bytes;// data size

//晶振频率修改时，RGB1W_CYC 和 PIOC_CYC 需要相应修改
#if    Mode_Select              // 方式1驱动RGB

    	RGB1W_COMMAND = RGB1W_CYC | RGB1W_CMD_RAM;// set bit cycle and PIOC start send

#else                           // 方式2驱动 RGB
	    static  u8 T0H_count = (( RGB_DI_T0H / PIOC_CYC ) - 3 ) / 3;          //计算公式( 1 + 3 * count + 2 ) * K = T ，count值不能为0，  K:指令周期
	    static  u8 T0L_count = (( RGB_DI_T0L / PIOC_CYC ) - 3 - 5 ) / 3;      //公式中的“ - 5 ”：判断过程所用指令所需指令周期
	    static  u8 T1H_count = (( RGB_DI_T1H / PIOC_CYC ) - 3 ) / 3;          // PIOC_Timer_48M ：48MHZ下指令周期
	    static  u8 T1L_count = (( RGB_DI_T1L / PIOC_CYC ) - 3 - 5 ) / 3;

        RGB1W_COMMAND = 0x30;

        PIOC->D8_DATA_REG4 =  T0H_count ? T0H_count : 1 ;  //
        PIOC->D8_DATA_REG5 =  T0L_count ? T0L_count : 1 ;  //
        PIOC->D8_DATA_REG6 =  T1H_count ? T1H_count : 1 ;  //
        PIOC->D8_DATA_REG7 =  T1L_count ? T1L_count : 1 ;  //
        //PIOC->D8_DATA_REG7 = 1 ;  // RGB_DI_T1L 偏小使 T1L_count小于0时，需将T1L_count设为1

#endif

//#ifdef   SYSCLK_FREQ_24MHz_HSI
//
//if(Mode_Select)   //方式1驱动
//         {
//             RGB1W_COMMAND = RGB1W_CYC_24M | RGB1W_CMD_RAM;// set bit cycle and PIOC start send
//         }
//         else           // 方式2驱动
//         {
//             RGB1W_COMMAND = 0x30;
//    static  u8 T0H_count=(( RGB_DI_T0H / PIOC_Timer_48M ) - 3 ) / 3;
//    static  u8 T0L_count=(( RGB_DI_T0L / PIOC_Timer_48M ) - 3 - 5 ) / 3;
//    static  u8 T1H_count=(( RGB_DI_T1H / PIOC_Timer_48M ) - 3 ) / 3;
//    static  u8 T1L_count=(( RGB_DI_T1L / PIOC_Timer_48M ) - 3 - 5 ) / 3;
//             PIOC->D8_DATA_REG4 =  T0H_count ? T0H_count : 1 ;  //
//             PIOC->D8_DATA_REG5 =  T0L_count ? T0L_count : 1 ;  //
//             PIOC->D8_DATA_REG6 =  T1H_count ? T1H_count : 1  ;  //
//             PIOC->D8_DATA_REG7 =  T1L_count ? T1L_count : 1  ;  //
//           printf("FREQ_24MHz\r\n");
//         }
//
//#else	// default is 48MHz
//	if(RGB_Class)   //兼容类型
//	 {
//	RGB1W_COMMAND = RGB1W_CYC_48M | RGB1W_CMD_RAM;// set bit cycle and PIOC start send
//	 }
//	else {
//	    RGB1W_COMMAND = 0x30;     // 非兼容，且晶振为48MHZ时
//
//	  static  u8 T0H_count=(( RGB_DI_T0H / PIOC_Timer_24M ) - 3 ) / 3;
//	  static  u8 T0L_count=(( RGB_DI_T0L / PIOC_Timer_24M ) - 3 - 5 ) / 3;
//	  static  u8 T1H_count=(( RGB_DI_T1H / PIOC_Timer_24M ) - 3 ) / 3;
//	  static  u8 T1L_count=(( RGB_DI_T1L / PIOC_Timer_24M ) - 3 - 5 ) / 3;
//        PIOC->D8_DATA_REG4 =  T0H_count ? T0H_count : 1 ;  //
//        PIOC->D8_DATA_REG5 =  T0L_count ? T0L_count : 1 ;  //
//        PIOC->D8_DATA_REG6 =  T1H_count ? T1H_count : 1  ;  //
//        PIOC->D8_DATA_REG7 =  T1L_count ? T1L_count : 1  ;  //
//    }
//
//
//#endif
}

uint8_t RGB1W_SendSFR_Wait( uint16_t total_bytes, uint8_t *p_source_addr ) {//SFR mode for 1~32 bytes data
// p_source_addr: point source data buffer start address, set NULL if copy into PIOC buffer already
	if ( total_bytes == 0 || total_bytes > RGB1W_SFR_SIZE ) return( RGB1W_ERR_PARA );
	RGB1W_SendSFR( total_bytes, p_source_addr );
	while ( ( PIOC->D8_SYS_CFG & RB_INT_REQ ) == 0 );//wait, PIOC request interrupt after finish
	return( PIOC->D8_CTRL_RD );//auto remove interrupt request after reading
}

uint8_t RGB1W_SendRAM_Wait( uint16_t total_bytes, uint8_t *p_source_addr ) {//RAM mode for 1~3072 bytes data
// p_source_addr: point source data buffer start address, set NULL if copy into PIOC buffer already
	if ( total_bytes == 0 || total_bytes > RGB1W_RAM_SIZE ) return( RGB1W_ERR_PARA );
	RGB1W_SendRAM( total_bytes, p_source_addr );
	while ( ( PIOC->D8_SYS_CFG & RB_INT_REQ ) == 0 );//wait, PIOC request interrupt after finish
	return( PIOC->D8_CTRL_RD );//auto remove interrupt request after reading
}

void RGB1W_Halt( void ) {//halt/sleep PIOC
	PIOC->D8_SYS_CFG &= ~ RB_MST_CLK_GATE;
}


void RGB_Single_Set( uint16_t RGB_Number, uint8_t Green, uint8_t Red, uint8_t Blue )  //单个RGB灯珠灯光数据设置
{
    RGBpbuf1[ RGB_Number ][ 0 ] = Green;

    RGBpbuf1[ RGB_Number ][ 1 ] = Red;

    RGBpbuf1[ RGB_Number ][ 2 ] = Blue;

}

void RGB_Whole_Set( uint8_t Green,uint8_t Red ,uint8_t Blue )  //单个RGB灯珠灯光数据设置
{

  for( uint16_t i = 0; i <  RGB_Total_Number ; i ++ ) //纯色
   {
      RGB_Single_Set(i , Green  , Red  ,Blue  );
   }

}


