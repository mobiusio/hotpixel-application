#ifndef _LED_H_
#define _LED_H_

//
// defines
//

#define RED_CHANNEL						0x01
#define GREEN_CHANNEL					0x02
#define BLUE_CHANNEL					0x04

#define RED_W_ADDR						0x60
#define GREEN_W_ADDR					0x61
#define BLUE_W_ADDR						0x62
#define	LED_SYSTICK_MAX				0x02 // 0x02 is too brigh to look at in a dark room
#define LED_SYSTICKS					4000 // 4096 // 8192 is ok from peripheral vision, might need to be increased
#define	LED_SIZE_X						8 
#define LED_SIZE_Y						8
#define LED_RANDOM_HIGH_CONTRAST 				1	
#define LED_RANDOM_HIGH_CONTRAST_SHIFT	2//3
//
// structures
//

struct led_pixel_8 {
	/* // reversing for litte endianness
	uint_8	flags;
	uint_8	red;
	uint_8	green;
	uint_8	blue;
	*/
	uint_8 blue;
	uint_8 green;
	uint_8 red;
	uint_8 flags;
};

union led_pixel {
	uint_32	raw;
	struct led_pixel_8 pixel;
};

//
// led functions
//

void led_init (void);
void led_random (void);
void led_symbol (uint_32 sym, uint_8 x, uint_8 y, uint_8 r, uint_8 g, uint_8 b);
void led_systick_handler (void);
void led_paint_cols (void); 
void led_paint_rows (void);
void led_all_off (void);

uint_8 led_color_adjust(uint_8 val, uint_8 channel);

#endif //_LED_H_
