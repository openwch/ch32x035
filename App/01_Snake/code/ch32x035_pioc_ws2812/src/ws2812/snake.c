#include "snake.h"
#include "stdlib.h"

/**
 * @brief 
 * 0 -> free
 * 1 -> body
 * 2 -> food
 * 3 -> wall
 */
uint8_t map[32][32] = {0};
coordinate_t snakeBody[1024];
volatile uint16_t snakeLength = 3;
volatile uint8_t snakeMoveDir = MOVE_GO;
volatile uint32_t snakeMoveTime = SNAKE_MOVE_SLOW;
volatile ws2812_byte_buffer_t ws2812_rgb_data;
volatile ws2812_hsv_t         ws2812_hsv_data;


/**
 * @brief 将HSV颜色空间转换为RGB颜色空间
 *      - 因为HSV使用起来更加直观、方便，所以代码逻辑部分使用HSV。但WS2812B RGB-LED灯珠的驱动使用的是RGB，所以需要转换。
 * 
 * @param  h HSV颜色空间的H：色调。单位°，范围0~360。（Hue 调整颜色，0°-红色，120°-绿色，240°-蓝色，以此类推）
 * @param  s HSV颜色空间的S：饱和度。单位%，范围0~100。（Saturation 饱和度高，颜色深而艳；饱和度低，颜色浅而发白）
 * @param  v HSV颜色空间的V：明度。单位%，范围0~100。（Value 控制明暗，明度越高亮度越亮，越低亮度越低）
 * @param  rgb_buffer RGB值的指针
 *
 * Wiki: https://en.wikipedia.org/wiki/HSL_and_HSV
 *
 */
void ws2812_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, ws2812_byte_buffer_t *rgb_buffer)
{
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    case 0:
        rgb_buffer->red = rgb_max;
        rgb_buffer->green = rgb_min + rgb_adj;
        rgb_buffer->blue = rgb_min;
        break;
    case 1:
        rgb_buffer->red = rgb_max - rgb_adj;
        rgb_buffer->green = rgb_max;
        rgb_buffer->blue = rgb_min;
        break;
    case 2:
        rgb_buffer->red = rgb_min;
        rgb_buffer->green = rgb_max;
        rgb_buffer->blue = rgb_min + rgb_adj;
        break;
    case 3:
        rgb_buffer->red = rgb_min;
        rgb_buffer->green = rgb_max - rgb_adj;
        rgb_buffer->blue = rgb_max;
        break;
    case 4:
        rgb_buffer->red = rgb_min + rgb_adj;
        rgb_buffer->green = rgb_min;
        rgb_buffer->blue = rgb_max;
        break;
    default:
        rgb_buffer->red = rgb_max;
        rgb_buffer->green = rgb_min;
        rgb_buffer->blue = rgb_max - rgb_adj;
        break;
    }
}


/**
 * @brief 
 * 
 * @param position 
 * @return uint16_t 
 * 
 *  31     0  15  *********************  255
 *  30     1  14                         254
 *  29     2  13                         253
 *  28     3  12       (part 3)          252
 *  27     4  11                         251
 *  26     5  10                         250
 *  25     6  9                          249
 *  24     7  8   *********************  255
 * 
 *  23     0  15  *********************  255
 *  22     1  14                         254
 *  21     2  13                         253
 *  20     3  12       (part 2)          252
 *  19     4  11                         251
 *  18     5  10                         250
 *  17     6  9                          249
 *  16     7  8   *********************  255
 * 
 *  15     0  15  *********************  255
 *  14     1  14                         254
 *  13     2  13                         253
 *  12     3  12       (part 1)          252
 *  11     4  11                         251
 *  10     5  10                         250
 *  9      6  9                          249
 *  8      7  8   *********************  255
 *    
 *  7      0  15  *********************  255
 *  6      1  14                         254
 *  5      2  13                         253
 *  4      3  12       (part 0)          252
 *  3      4  11                         251
 *  2      5  10                         250
 *  1      6  9                          249
 *  0      7  8   *********************  255
 * (y)
 *         0  1  2  3  4  5 ***********  31(x)
 */
uint16_t coordinate_convert(coordinate_t *position)
{
    uint16_t index = 0;
    if(position->x>31 || position->y>31)
    {
        return index;
    }
    uint8_t partNUM = position->y/8;
    if(position->x%2 == 1) //odd
    {
        index = (position->x*8) + (position->y%8);
    }
    else // even
    {
        index = (position->x+1)*8 - (position->y%8 + 1);
    }

    index += partNUM*256;
    return index;
}

uint16_t creat_food(void)
{
    uint16_t index = 0;

    uint32_t time = 0;
    coordinate_t food = 
    {
        .x = 0,
        .y = 0 
    };

    do
    {
        food.x = rand()%32;
        food.y = rand()%32;
        // printf("food.x = %d, food.y = %d \r\n",food.x,food.y);
        time++;
    } while ( (map[food.x][food.y]!=0) || (time>60000));

    map[food.x][food.y] = MAP_FOOD;

    index = coordinate_convert(&food);
    // RGB_Single_Set(index, 64, 0 ,0 );
    return index;
}

void snap_init(void)
{
    uint16_t i =0;
    snakeLength = 3;
    snakeMoveDir = MOVE_GO;
    

    for(i=0; i<32; i++)
    {
        for(uint8_t j=0; j<32; j++)
        {
            map[i][j] = MAP_FREE;
        }
    }

    snakeBody[0].x = 16;
    snakeBody[0].y = 17;
    map[snakeBody[0].x][snakeBody[0].y] = MAP_BODY;

    snakeBody[1].x = 16;
    snakeBody[1].y = 16;
    map[snakeBody[1].x][snakeBody[1].y] = MAP_BODY;

    snakeBody[2].x = 16;
    snakeBody[2].y = 15;
    map[snakeBody[2].x][snakeBody[2].y] = MAP_BODY;

    for(i=0; i<32; i++) // init wall around
    {
        map[0][i] = MAP_WALL;
        map[31][i] = MAP_WALL;
        map[i][0] = MAP_WALL;
        map[i][31] = MAP_WALL;
    }

    creat_food();

    updata_rgb_by_map(map);
}

void updata_rgb_by_map(uint8_t (*map)[32])
{
    coordinate_t pos;
    uint16_t index = 0;
    ws2812_hsv_t hsv_data;
    ws2812_byte_buffer_t rgb_data;

    for(uint8_t i=0; i<32; i++)
    {
        for(uint8_t j=0; j<32; j++)
        {
            pos.x = i;
            pos.y = j;
            index = coordinate_convert(&pos);

            switch(map[pos.x ][pos.y])
            {
                case MAP_FREE:
                    RGB_Single_Set(index, 0, 0 ,0 );
                    break;
                case MAP_WALL:
                    hsv_data.hue = 200;
                    hsv_data.saturation = 100;
                    hsv_data.value = 10;

                    ws2812_hsv2rgb(hsv_data.hue, hsv_data.saturation, hsv_data.value, &rgb_data);
                    RGB_Single_Set(index, rgb_data.green, rgb_data.red ,rgb_data.blue );
                    break;
                case MAP_BODY:
                    RGB_Single_Set(index, 10, 0 ,0 );
                    break;
                case MAP_FOOD:
                    RGB_Single_Set(index, 0, 10 ,0 );
                    break;
                default:
                    break;
            }
        }
    }
}


void snake_move(void)
{
    uint16_t i = 0;
    coordinate_t temp1,temp2;
    temp1.x = snakeBody[0].x;
    temp1.y = snakeBody[0].y;

    // snake last body free
    map[snakeBody[snakeLength-1].x][snakeBody[snakeLength-1].y] = MAP_FREE;

    // snake head move
    switch (snakeMoveDir)
    {
    case MOVE_GO:
        snakeBody[0].y++;
        if(snakeBody[0].y>31) snakeBody[0].y = 31;
        break;

    case MOVE_BACK:
        snakeBody[0].y--;
        if(snakeBody[0].y<0) snakeBody[0].y = 0;
        break;    

    case MOVE_LEFT:
        snakeBody[0].x--;
        if(snakeBody[0].x<0) snakeBody[0].x = 0;
        break;  

    case MOVE_RIGHT:
        snakeBody[0].x++;
        if(snakeBody[0].x>31) snakeBody[0].x = 31;
        break;  
    
    default:
        break;
    }


    for( i=1; i<snakeLength; i++)
    {
        temp2.x = snakeBody[i].x;
        temp2.y = snakeBody[i].y;
        snakeBody[i].x = temp1.x;
        snakeBody[i].y = temp1.y;
        temp1.x = temp2.x;
        temp1.y = temp2.y;
    }
    

    // eat food
    if(map[snakeBody[0].x][snakeBody[0].y] == MAP_FOOD)
    {
        printf("eat food\r\n");
        snakeBody[snakeLength] = snakeBody[snakeLength-1];
        snakeLength++;
        creat_food();
    }
    if( (map[snakeBody[0].x][snakeBody[0].y] == MAP_WALL) || (map[snakeBody[0].x][snakeBody[0].y] == MAP_BODY))
    {
        printf("hit the wall\r\n");
        snap_init();
    }

    for(i=0; i<snakeLength; i++)
    {
        map[snakeBody[i].x][snakeBody[i].y] = MAP_BODY;
    }

    updata_rgb_by_map(map);
}


void restart_snake(void)
{
    for(uint16_t i=0; i<1024; i++)
    {
        RGB_Single_Set(1, 64, 0 ,0 );
    }

}
