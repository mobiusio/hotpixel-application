#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "types.h"
#include "derivative.h" /* include peripheral declarations */
#include "user_config.h"
#include "usb_cdc.h"        /* USB CDC Class Header File */
#include "virtual_com.h"
#include "led.h"
#include "ui.h"
#include "ff.h"
#include "diskio.h"
#include "fileio.h"

volatile uint_8 ui_has_data_to_process = FALSE;

static uint_8 ui_rx_buf[UI_RX_BUFFER_LENGTH];
static uint_8 ui_tx_buf[UI_TX_BUFFER_LENGTH];

volatile static uint_32 ui_rx_buf_idx = 0;
volatile uint_32	ui_demo_mode = UI_DEMO_NONE;
static uint_32 ui_screen_buffer[64];
volatile uint_32 usb_io_led = 0; // be able to flash LED when there is USB traffic

extern union led_pixel pixels[LED_SIZE_X][LED_SIZE_Y]; // from led.c
extern uint_8 g_curr_recv_buf[DATA_BUFF_SIZE]; // from virtual_com.c
extern uint_8 g_curr_send_buf[DATA_BUFF_SIZE]; // from virtual_com.c

volatile uint_32	ui_delay_counter = 0;

char ui_path[UI_PATH_LENGTH] = {'/', 0x00}; // create a path that is null terminated

uint_32 nyan_frame0[] = {
0x00000000, 0x00000000, 0x00000000, 0x00cc33cc, 0x00ffff00, 0x00ff0000, 0x00000000, 0x00000000, 
0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00cc33cc, 0x00ffff00, 0x00ff0000, 0x00000000, 
0x00000000, 0x00000000, 0x00000000, 0x00cc00ff, 0x00ffff00, 0x00ff0000, 0x00000000, 0x00000000, 
0x00000000, 0x00000000, 0x00333333, 0x00ff9999, 0x00ff9999, 0x00ff9999, 0x00000000, 0x00000000, 
0x00000000, 0x00000000, 0x00000000, 0x00ff9999, 0x00ff9999, 0x00ff9999, 0x00000000, 0x00000000, 
0x00000000, 0x00000000, 0x00333333, 0x00333333, 0x00333333, 0x00ff9999, 0x00000000, 0x00000000, 
0x00000000, 0x00000000, 0x00000000, 0x00333333, 0x00333333, 0x00000000, 0x00000000, 0x00000000, 
0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,   
};

uint_32 nyan_frame1[] = {
0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00cc00ff, 0x00ffff00, 0x00ff0000, 0x00000000, 
0x00000000, 0x00000000, 0x00000000, 0x00cc00ff, 0x00ffff00, 0x00ff0000, 0x00000000, 0x00000000, 
0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00cc00ff, 0x00ffff00, 0x00ff0000, 0x00000000, 
0x00000000, 0x00000000, 0x00000000, 0x00333333, 0x00ff9999, 0x00ff9999, 0x00ff9999, 0x00000000, 
0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00ff9999, 0x00ff9999, 0x00ff9999, 0x00000000, 
0x00000000, 0x00000000, 0x00000000, 0x00333333, 0x00333333, 0x00333333, 0x00ff9999, 0x00000000, 
0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00333333, 0x00333333, 0x00000000, 0x00000000, 
0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
};

uint_8 ui_send_str (uint_8 *buf, uint_32 length) {
	/*
	 * this function attempts to send a string to the CDC port using UI_SEND_SIZE
	 * increments
	 */
	uint_8 status;
	
	while (length > 0) {
		if (length >= UI_SEND_SIZE) {
			status = USB_Class_CDC_Interface_DIC_Send_Data(CONTROLLER_ID, buf, UI_SEND_SIZE);
			length -= UI_SEND_SIZE;
			buf += UI_SEND_SIZE;
		}
		else {
			status = USB_Class_CDC_Interface_DIC_Send_Data(CONTROLLER_ID, buf, length);
			length = 0;
		}
		
		ui_delay(25); // 25ms to allow the transmit queue to be flushed
		
		if (status != USB_OK) {
			// return if there was a problem
			return(status);
		}		
	}
	
	return(E_OK);
}

uint_8 ui_handle_input (uint_8 *inptr, uint_8 **outptr, uint_32 *length) {
	//
	// This function handles the input from the vcom level and generates a reply,
	// it uses a pointer to the pointer of the output buffer to specify 
	// the data to be written back. Also the location pointed to by length
	// gets filled with the length of data to be written back. This function
	// will set a flag to let the main loop know there are data to be processed.
	// 
	uint_32 i, inlength = *length;
	uint_8 *ptr = (uint_8 *)(&g_curr_send_buf);
	
	usb_io_led = UI_LED_CYCLES;
	
	*length = 0;
	
	for (i = 0; i < inlength; i++) {
		if (i < DATA_BUFF_SIZE) {
			g_curr_send_buf[(*length)++] = inptr[i];
		}
		
		if ((ui_rx_buf_idx < UI_RX_BUFFER_LENGTH) && (ui_has_data_to_process == FALSE)) {
			// checking ui_has_data_to_process here so we can reuse ui_rx_buf for TX stuff too
			ui_rx_buf[ui_rx_buf_idx++] = inptr[i];
		}
		
		switch (inptr[i]) {
			case UI_KEY_NL:
		  case UI_KEY_CR:
				// flag the string to be processed, need to check here if it is a multi-line set string command
				if (!strncmp((const char *)ui_rx_buf, UI_SET_SCREEN_STR, strlen(UI_SET_SCREEN_STR))) {
					if (ui_rx_buf_idx < UI_SET_SCREEN_LEN) {
						sprintf((char *)g_curr_send_buf, "\r\n");
            *length = strlen((const char *)g_curr_send_buf);
            continue;
          }
					else {
						ui_has_data_to_process = TRUE;
					}
				}
				else {
					ui_has_data_to_process = TRUE;
				}
			
			case UI_KEY_BACKSPACE:
				if (ui_rx_buf_idx) {
					ui_rx_buf_idx--;
				}
			
			default:
				break;
		}
	}
	
	*outptr = ptr;
	
	return(E_OK);
}

uint_8 ui_process_string (void) {
	/*
	 * this function checks to see if ui_has_data_to_process indicates
	 * that there are data to be processed, then handles those data.
	 * this function is intended to be run from main loop.
	 */
	
	FRESULT fresult;
	FIL fil;
	UINT w;
	
  if (ui_has_data_to_process == FALSE) {
    return(E_OK); // nothing to do
  }
  
  // select what to do
  if (!strncmp((const char *)ui_rx_buf, UI_HELP_STR, strlen(UI_HELP_STR))) {
    ui_send_str(UI_HELP, strlen(UI_HELP));
  }
	else if (!strncmp((const char *)ui_rx_buf, UI_SET_PIXEL_STR, strlen(UI_SET_PIXEL_STR))) {
		ui_set_pixel(ui_rx_buf, ui_rx_buf_idx);
	}
	else if (!strncmp((const char *)ui_rx_buf, UI_SET_SCREEN_STR, strlen(UI_SET_SCREEN_STR))) {
		ui_set_screen(ui_rx_buf, ui_rx_buf_idx);
	}
  else if (!strncmp((const char *)ui_rx_buf, UI_DEMO_RANDOM_STR, strlen(UI_DEMO_RANDOM_STR))) {
		ui_demo_mode = (uint_8)UI_DEMO_RANDOM;
		ui_send_str("\r\nenabling random color demo (high contrast)\r\n", strlen("\r\nenabling random color demo (high contrast)\r\n"));
  }
  else if (!strncmp((const char *)ui_rx_buf, UI_DEMO_NYAN_STR, strlen(UI_DEMO_NYAN_STR))) {
    ui_demo_mode = (uint_8)UI_DEMO_NYAN;
		ui_send_str("\r\nenabling nyan cat demo mode\r\n", strlen("\r\nenabling nyan cat demo mode\r\n"));
	}
  else if (!strncmp((const char *)ui_rx_buf, UI_DEMO_NONE_STR, strlen(UI_DEMO_NONE_STR))) {
    memset(pixels, 0x00, sizeof(pixels));
    ui_demo_mode = (uint_8)UI_DEMO_NONE;
		ui_send_str("\r\ndisabeling demo mode\r\n", strlen("\r\ndisabeling demo mode\r\n"));
  }
	else if (!strncmp((const char *)ui_rx_buf, UI_FS_PWD, strlen(UI_FS_PWD))) {
		// sprintf doesn't null terminate?
		sprintf((char *)ui_rx_buf, "\r\n%s\r\n\0", ui_path);
		ui_send_str(ui_rx_buf, strlen((const char *)ui_rx_buf));
	}
	else if (!strncmp((const char *)ui_rx_buf, UI_FS_LS, strlen(UI_FS_LS))) {
		//
		// THIS FUNCTION HAS PROBLEMS AFTER THE FIRST SEND (doesn't print header fully)
		//
		ui_handle_ls(ui_path, TRUE);
	}
	else if (!strncmp((const char *)ui_rx_buf, "dump", 4)) {
		ui_send_str("\r\ndumping screen to /dump.bin\r\n", strlen("\r\ndumping screen to /dump.bin\r\n"));
		fresult = f_open(&fil, "/dump.bin", FA_WRITE | FA_CREATE_ALWAYS);
		if (fresult == FR_OK) {
			fresult = f_write(&fil, &pixels, sizeof(pixels), &w);
			f_close(&fil);
		}
		else {
			ui_send_str("problem with f_open()\r\n", strlen("problem with f_open()\r\n"));
		}
	}
	else if (!strncmp((const char *)ui_rx_buf, "play", 4)) {
		ui_send_str("\r\nplaying '/default.scr'\r\n", strlen("\r\nplaying '/default.scr'\r\n"));
		fileio_play_file("/DEFAULT.SCR");
	}
	else if (!strncmp((const char *)ui_rx_buf, UI_FS_CD, strlen(UI_FS_CD))) {
		
	}
  
  ui_rx_buf_idx = 0;
  ui_has_data_to_process = FALSE; // clear process
  return(E_OK);
}

uint_8 ui_set_screen (uint_8 *ptr, uint_32 idx) {
	/*
	 * this function processes the 'set string' directive
	 */
	
	// returns 0 if no error, format:
	// set screen #rrggbb,#rrggbb,#rrggbb (64 total sets)
	uint_8 *pptr = ptr + 11;
	uint_32 i, count = 0; // will count to 63
	uint_8 buf[8] = {0,0,0,0,0,0,0,0};

	while ((count < 64) && (pptr < ptr+idx)) {
		if (*pptr == '#') {
			// find a color code
		  memcpy(buf, pptr+1, 6); // copy color code
			sscanf((const char *)buf, "%06x", &i);
			ui_screen_buffer[count] = i;
			count++;
			pptr += 6;
		}
		else {
			pptr++;
		}
	}
	
	memcpy((uint_32 *)pixels, (uint_32 *)ui_screen_buffer, sizeof(ui_screen_buffer));
	
	if (count != 64) {
		ui_send_str("\r\nset screen error\r\n", strlen("\r\nset screen error\r\n"));
		return(E_BOUNDS);
	}
	else {	
		ui_send_str("\r\nset screen complete\r\n", strlen("\r\nset screen complete\r\n"));
		return(E_OK);
	}
}

uint_8 ui_set_pixel (uint_8 *ptr, uint_32 idx) {
	/*
	 * this function sets one pixel on the screen based on 
	 * 'set pixel x,y #rrggbb' format
	 */
	
	uint_8 x, y, r, g, b;
	uint_32 i;
	
	// set pixel x,y #zzzzzz<ret>
  x = ui_rx_buf[10]-'0';
  y = ui_rx_buf[12]-'0';
  if (ui_rx_buf_idx < 20) {
    r = 0;
    g = 0;
    b = 0;
  }
  else {
    ui_rx_buf[21] = 0; // enforce null termination
    sscanf((const char *)&ui_rx_buf[15], "%06x", &i);
    r = 0xFF & (i >> 16);
    g = 0xFF & (i >> 8);
    b = 0xFF & (i);
  }
	
	if ((x < LED_SIZE_X) && (y < LED_SIZE_Y)) {
    pixels[x][y].pixel.red = r;
    pixels[x][y].pixel.green = g;
    pixels[x][y].pixel.blue = b;
  }
	
	sprintf((char *)ui_rx_buf, "\r\npixel x=%d,y=%d (r,g,b)=(0x%02x, 0x%02x, 0x%02x)\r\n", x, y, r, g, b);
	ui_send_str(ui_rx_buf, strlen((const char *)ui_rx_buf));
	
	return(E_OK);
}

uint_8 ui_handle_ls (char *path, uint_8 is_dir) {
        /*
         * this function handles printing out contents of current path,
         * is_dir is either TRUE or FALSE
         */
        
        DIR dir;
        FILINFO filinfo;
        FRESULT fresult; 
        
        // should check if there is a fatfs macro to check if path is dir/not
        if (is_dir == TRUE) {
                sprintf((char *)ui_tx_buf, "\r\ncontents of: %s\r\n\0", path);
                ui_send_str(ui_tx_buf, strlen((const char *)ui_tx_buf));
                fresult = f_opendir(&dir, path);
                if (fresult == FR_OK) {
                        fresult = f_readdir(&dir, &filinfo);
                        while ((fresult == FR_OK) && (filinfo.fname[0])) {
                                // print out details
                                sprintf((char *)ui_tx_buf, "%s -- %d bytes\r\n\0", filinfo.fname, filinfo.fsize);
                                ui_send_str(ui_tx_buf, strlen((const char *)ui_tx_buf));
                                fresult = f_readdir(&dir, &filinfo);
                        }
                }
                else {
                        return((uint_8) fresult);
                }
        }
        return(E_OK);
}

void ui_demo_nyan (void) {
	/*
	 * this function alternates the nyan cat demo frames
	 */
	
	static uint_8 frame = 0;
	if (!frame) {
		memcpy(pixels, nyan_frame0, sizeof(pixels));
		frame = 1;
	}
	else {
		memcpy(pixels, nyan_frame1, sizeof(pixels));
		frame = 0;
	}
}

void ui_delay (uint_32 ms) {
	/*
	 * this function uses systick to create a small delay
	 */
	ui_delay_counter = ms*4;
	
	while (ui_delay_counter); // wait 	
}

void ui_delay_set (uint_32 ms) {
	/*
	 * this function sets the countdown delay to wait
	 */
	ui_delay_counter = ms*4;
}

void ui_delay_wait (void) {
	/*
	 * this function waits for the delay counter to expire
	 */
	while (ui_delay_counter); // wait
}

