#include "arm_math.h"
//command ids
#define CHECK_STATUS 		0
#define TAKE_PICTURE 		1
#define GET_PICTURE_SIZE 	3
#define GET_PICTURE 		5
#define SET_CONTRAST 		6
#define SET_BRIGHTNESS 		7
#define SET_EXPOSURE 		8
#define SET_SLEEP_TIME 		9
//Picture
#define IMAGE_SIZE 75000
extern uint8_t IMG_PICTURE[IMAGE_SIZE];
extern uint16_t current_image_size;
// IMG system function declaration
uint32_t image_sendCommand( uint8_t command, uint8_t param );
uint32_t image_getPicture( uint8_t IMG_param );
