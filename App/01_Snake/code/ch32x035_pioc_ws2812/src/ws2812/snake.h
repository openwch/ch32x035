#ifndef _SNAKE_H_
#define _SNAKE_H_

#include "debug.h"
#include "RGB1W.h"


#define  MOVE_GO     0
#define  MOVE_BACK   1
#define  MOVE_LEFT   2
#define  MOVE_RIGHT  3


#define MAP_FREE  0
#define MAP_BODY  1
#define MAP_FOOD  2
#define MAP_WALL  3

#define SNAKE_MOVE_SLOW   80
#define SNAKE_MOVE_QUICK  20

extern volatile uint8_t snakeMoveDir;
extern volatile uint32_t snakeMoveTime;

typedef struct 
{
    uint8_t green;
    uint8_t red;
    uint8_t blue;
} ws2812_byte_buffer_t;

typedef struct 
{
    uint16_t hue;
    uint8_t  saturation;
    uint8_t  value;
}ws2812_hsv_t;


typedef struct 
{
    int8_t x;
    int8_t y;
}coordinate_t;

// typedef struct snakes
// {
//     coordinate_t posistion;
//     struct snakes *next;
// }snake_t;

void snap_init(void);
void updata_rgb_by_map(uint8_t (*map)[32]);
uint16_t coordinate_convert(coordinate_t *position);
uint16_t creat_food(void);
void snake_move(void);

#endif /* end of snake.h*/
