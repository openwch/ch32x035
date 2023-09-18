#include "button.h"
#include "multi_button.h"


Button buttonLeft;
Button buttonRight;
Button buttonGo;
Button buttonBack;

uint8_t pdoIndex = 0;

/**
 * @brief init button gpio
 * 
 */
void my_button_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};
    RCC_APB2PeriphClockCmd(BUTTON_LEFT_CLK|BUTTON_RIGHT_CLK|BUTTON_GO_CLK|BUTTON_BACK_CLK, ENABLE );
    GPIO_InitStructure.GPIO_Pin = BUTTON_LEFT_PIN;   /* button left */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(BUTTON_LEFT_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = BUTTON_RIGHT_PIN;  /* button right */
    GPIO_Init(BUTTON_RIGHT_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = BUTTON_GO_PIN;   /* button go */
    GPIO_Init(BUTTON_GO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = BUTTON_BACK_PIN;   /* button back */
    GPIO_Init(BUTTON_BACK_PORT, &GPIO_InitStructure);
}

uint8_t read_button_pin(uint8_t button_id)
{
    switch(button_id)
    {
        case BUTTON_LEFT_ID:
            return GPIO_ReadInputDataBit(BUTTON_LEFT_PORT,BUTTON_LEFT_PIN);
            break;

        case BUTTON_RIGHT_ID:
            return GPIO_ReadInputDataBit(BUTTON_RIGHT_PORT,BUTTON_RIGHT_PIN);
            break;

        case BUTTON_GO_ID:
            return GPIO_ReadInputDataBit(BUTTON_GO_PORT,BUTTON_GO_PIN);
            break;

        case BUTTON_BACK_ID:
            return GPIO_ReadInputDataBit(BUTTON_BACK_PORT,BUTTON_BACK_PIN);
            break;

        default:
            return 0;
            break;
    }
}


void left_button_callback(void* btn)
{
    uint32_t buttonEventValue = get_button_event((struct Button*)btn);

    switch (buttonEventValue)
    {
    case PRESS_DOWN:
        if(snakeMoveDir != MOVE_RIGHT)
        {
            snakeMoveDir = MOVE_LEFT;
        }
        
        printf("button %d PRESS_DOWN\r\n",((Button*)btn)->button_id);
        break;

    case LONG_PRESS_HOLD:
        if(snakeMoveDir != MOVE_RIGHT)
        {
            snakeMoveDir = MOVE_LEFT;
        }
        snakeMoveTime = SNAKE_MOVE_QUICK;
        printf("button %d LONG_PRESS_HOLD\r\n",((Button*)btn)->button_id);
        break;

    case PRESS_UP:
        snakeMoveTime = SNAKE_MOVE_SLOW;
        printf("button %d PRESS_UP\r\n",((Button*)btn)->button_id);
        break;
    
    default:
        break;
    }
}

void right_button_callback(void* btn)
{
    uint32_t buttonEventValue = get_button_event((struct Button*)btn);

    switch (buttonEventValue)
    {
    case PRESS_DOWN:
        if(snakeMoveDir != MOVE_LEFT)
        {
            snakeMoveDir = MOVE_RIGHT;
        }
        
        printf("button %d PRESS_DOWN\r\n",((Button*)btn)->button_id);
        break;

    case LONG_PRESS_HOLD:
        if(snakeMoveDir != MOVE_LEFT)
        {
            snakeMoveDir = MOVE_RIGHT;
        }
        snakeMoveTime = SNAKE_MOVE_QUICK;
        printf("button %d LONG_PRESS_HOLD\r\n",((Button*)btn)->button_id);
        
        break;

    case PRESS_UP:
        snakeMoveTime = SNAKE_MOVE_SLOW;
        printf("button %d PRESS_UP\r\n",((Button*)btn)->button_id);
        break;

    default:
        break;
    }
}

void go_button_callback(void* btn)
{
    uint32_t buttonEventValue = get_button_event((struct Button*)btn);

    switch (buttonEventValue)
    {
    case PRESS_DOWN:
        if(snakeMoveDir != MOVE_BACK)
        {
            snakeMoveDir = MOVE_GO;
        }
        
        printf("button %d PRESS_DOWN\r\n",((Button*)btn)->button_id);
        break;

    case LONG_PRESS_HOLD:
        if(snakeMoveDir != MOVE_BACK)
        {
            snakeMoveDir = MOVE_GO;
        }
        snakeMoveTime = SNAKE_MOVE_QUICK;
        printf("button %d LONG_PRESS_HOLD\r\n",((Button*)btn)->button_id);
        
        break;

    case PRESS_UP:
        snakeMoveTime = SNAKE_MOVE_SLOW;
        printf("button %d PRESS_UP\r\n",((Button*)btn)->button_id);
        break;

    default:
        break;
    }
}

void back_button_callback(void* btn)
{
    uint32_t buttonEventValue = get_button_event((struct Button*)btn);

    switch (buttonEventValue)
    {
    case PRESS_DOWN:
        if(snakeMoveDir != MOVE_GO)
        {
            snakeMoveDir = MOVE_BACK;
        }
        printf("button %d PRESS_DOWN\r\n",((Button*)btn)->button_id);
        break;

    case LONG_PRESS_HOLD:
        if(snakeMoveDir != MOVE_GO)
        {
            snakeMoveDir = MOVE_BACK;
        }
        snakeMoveTime = SNAKE_MOVE_QUICK;
        printf("button %d LONG_PRESS_HOLD\r\n",((Button*)btn)->button_id);
        break;

    case PRESS_UP:
        snakeMoveTime = SNAKE_MOVE_SLOW;
        printf("button %d PRESS_UP\r\n",((Button*)btn)->button_id);
        break;

    default:
        break;
    }
}

void my_button_init(void)
{
    my_button_gpio_init();
    Delay_Ms(20);

    /* init button object*/
    button_init(&buttonLeft,read_button_pin,0,BUTTON_LEFT_ID);
    button_attach(&buttonLeft,PRESS_DOWN,left_button_callback); 
    // button_attach(&buttonLeft,SINGLE_CLICK,left_button_callback); 
    button_attach(&buttonLeft,LONG_PRESS_HOLD,left_button_callback);
    button_attach(&buttonLeft,PRESS_UP,right_button_callback);

    button_init(&buttonRight,read_button_pin,0,BUTTON_RIGHT_ID);
    button_attach(&buttonRight,PRESS_DOWN,right_button_callback); 
    // button_attach(&buttonRight,SINGLE_CLICK,right_button_callback);
    button_attach(&buttonRight,LONG_PRESS_HOLD,right_button_callback);
    button_attach(&buttonRight,PRESS_UP,right_button_callback);
    

    button_init(&buttonGo,read_button_pin,0,BUTTON_GO_ID);
    button_attach(&buttonGo,PRESS_DOWN,go_button_callback); 
    // button_attach(&buttonGo,SINGLE_CLICK,go_button_callback);
    button_attach(&buttonGo,LONG_PRESS_HOLD,go_button_callback);
    button_attach(&buttonGo,PRESS_UP,right_button_callback);


    button_init(&buttonBack,read_button_pin,0,BUTTON_BACK_ID);
    button_attach(&buttonBack,PRESS_DOWN,back_button_callback); 
    // button_attach(&buttonBack,SINGLE_CLICK,back_button_callback);
    button_attach(&buttonBack,LONG_PRESS_HOLD,back_button_callback);
    button_attach(&buttonBack,PRESS_UP,right_button_callback);


    button_start(&buttonBack);
    button_start(&buttonGo);
    button_start(&buttonLeft);
    button_start(&buttonRight);
}




