#ifndef _BUTTON_H_
#define _BUTTON_H_

#include "debug.h"
#include "snake.h"

/* left button  -> PB10 */
#define  BUTTON_LEFT_CLK    RCC_APB2Periph_GPIOB
#define  BUTTON_LEFT_PORT   GPIOB
#define  BUTTON_LEFT_PIN    GPIO_Pin_10

/* right button -> PB8 */
#define  BUTTON_RIGHT_CLK   RCC_APB2Periph_GPIOB
#define  BUTTON_RIGHT_PORT  GPIOB
#define  BUTTON_RIGHT_PIN   GPIO_Pin_8

/* go button  -> PB9 */
#define  BUTTON_GO_CLK    RCC_APB2Periph_GPIOB
#define  BUTTON_GO_PORT   GPIOB
#define  BUTTON_GO_PIN    GPIO_Pin_9

/* back button  -> PB7 */
#define  BUTTON_BACK_CLK    RCC_APB2Periph_GPIOB
#define  BUTTON_BACK_PORT   GPIOB
#define  BUTTON_BACK_PIN    GPIO_Pin_7


/* button id enum */
typedef enum
{
    BUTTON_LEFT_ID = 0,
    BUTTON_RIGHT_ID,
    BUTTON_GO_ID,
    BUTTON_BACK_ID
}button_id_t;

extern void my_button_init(void);

#endif /* end of button.h*/
