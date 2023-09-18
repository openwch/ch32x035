#include "debug.h"
#include "usbpd_sink.h"
#include "usbpd_def.h"
#include "RGB1W.h"
#include "snake.h"
#include "button.h"


/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{

    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);

    Delay_Ms(1000);
    
    my_button_init();
    usbpd_sink_init();

    RGB1W_Init();
    snap_init();


    while(!usbpd_sink_get_ready())
    {
        usbpd_sink_set_request_fixed_voltage(REQUEST_20v);
    }


    while(1)
    {
        // printf("D8_DATA_REG11 : %x\r\n",PIOC->D8_DATA_REG11);
        // printf("stat = %x\r\n",stat);
        if(stat != 0xff)
        {
            if ( stat < 0x80 ) 
            {//got status in interrupt
                if ( stat == RGB1W_ERR_OK ){}// printf("1-wire finished\r\n");
                else 
                {
                    printf("1-wire error %02x\r\n", stat);
                }
                stat = 0x80;//free  
                // Delay_Ms(500);//wait then next
            }

            if(stat == 0x80)
            {
                stat = 0xff;//wait

                RGB1W_SendRAM( sizeof(RGBpbuf1), RGBpbuf1[0] );      // 灯效数据存储在RAM中
            }
        }

        if ( PIOC->D8_SYS_CFG & RB_INT_REQ )
        {//query if disable interrupt
            stat = PIOC->D8_CTRL_RD;//auto remove interrupt request after reading
        }

    }
}   





