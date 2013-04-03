#ifndef _UI_H_
#define _UI_H_

#define UI_RX_BUFFER_LENGTH	1024
#define UI_TX_BUFFER_LENGTH	256
#define UI_SEND_SIZE				64
#define UI_KEY_BACKSPACE		0x08
#define UI_KEY_RETURN				0x0D
#define UI_KEY_NL						'\n'
#define UI_KEY_CR						'\r'

#define UI_HELP_STR					"help"
#define UI_SET_PIXEL_STR		"set pixel"
#define UI_SET_SCREEN_STR		"set screen"
#define UI_SET_SCREEN_LEN		524 // minimum length for 
#define UI_DEMO_NONE_STR		"demo none"
#define UI_DEMO_NYAN_STR		"demo nyan"
#define UI_DEMO_RANDOM_STR	"demo random"

#define UI_DEMO_NONE				0x00
#define UI_DEMO_RANDOM			0x01
#define UI_DEMO_RANDOM_RATE	4096
#define UI_DEMO_NYAN				0x02
#define UI_DEMO_NYAN_RATE		3000

#define UI_LED_CYCLES				1024

#define UI_HELP							"\r\n\r\nHOTPIXEL v00.00.01 commands: \r\n\r\nset pixel x,y #rrggbb (in hex) -- to set pixel color\r\n" \
														"demo nyan -- nyan cat demo mode\r\n" \
														"demo random -- random, high contrast demo\r\n" \
														"demo none -- turn off demo mode\r\n" \
														"help -- this menu\r\n\r\n"
													
#define UI_PATH_LENGTH			64		
#define UI_FS_LS						"ls"
#define UI_FS_PWD						"pwd"
#define UI_FS_CD						"cd"


uint_8 ui_send_str (uint_8 *buf, uint_32 length);
uint_8 ui_handle_input (uint_8 *inptr, uint_8 **outptr, uint_32 *length);
void ui_demo_nyan (void);
uint_8 ui_set_screen (uint_8 *ptr, uint_32 idx);
uint_8 ui_set_pixel (uint_8 *ptr, uint_32 idx);
uint_8 ui_process_string (void);
void ui_delay (uint_32 ms);
void ui_delay_set (uint_32 ms);
void ui_delay_wait (void);

uint_8 ui_handle_ls (char *path, uint_8 is_dir);

#endif // _UI_H_
