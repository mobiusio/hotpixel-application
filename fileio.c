#include <stdlib.h>
#include <string.h>
#include "types.h"
#include "derivative.h" /* include peripheral declarations */
#include "led.h"
#include "ui.h"
#include "ff.h"
#include "diskio.h"
#include "fileio.h"

extern union led_pixel pixels[LED_SIZE_X][LED_SIZE_Y]; // imported from led.c
#define	F	"/DEFAULT.SCR"

// buffer to hold the largest object
//uint_8 fileio_buf[sizeof(struct fileio_opcode_screen_64)];
//uint_8 fileio_buf[0x104];
static uint_8 fileio_buf[1024];

void fileio_play_file (uint_8 *filename) {
	FRESULT fresult;
	FILINFO filinfo;
	FIL fil;
	UINT w;
	uint_32 idx = 0;
	//uint_8  wat[4];
	
	struct fileio_hdr *hdr = (struct fileio_hdr *)fileio_buf;
	struct fileio_opcode_screen_64 *screen_64 = (struct fileio_opcode_screen_64 *)fileio_buf;
	struct fileio_opcode_delay *delay = (struct fileio_opcode_delay *)fileio_buf;
	
	memset(fileio_buf, 0x00, 1024); // not sure if necessary
	fresult = f_stat(F, &filinfo);
	
	if (fresult != FR_OK) {
		ui_send_str("\r\ncould not stat file\r\n", strlen("\r\ncould not stat file\r\n"));
		return;
	}
	
	fresult = f_open(&fil, F, FA_READ);
	
	if (fresult != FR_OK) {
		ui_send_str("\r\ncould not open file\r\n", strlen("\r\ncould not open file\r\n"));
		return; //error
	}
	
	while (idx < filinfo.fsize) {
	//while (fresult == FR_OK) {
		//fresult = f_read(&fil, fileio_buf, sizeof(struct fileio_opcode_screen_64), &w);
		//idx += w;
		//memcpy(&pixels, screen_64->data, sizeof(pixels));
		fresult = f_read(&fil, fileio_buf, sizeof(struct fileio_hdr), &w);
		idx += w;
		if (hdr->magic == FILEIO_CMD_MAGIC) { 
			if (hdr->opcode == FILEIO_OPCODE_S64) {
				fresult = f_read(&fil, fileio_buf + sizeof(struct fileio_hdr), 
					sizeof(struct fileio_opcode_screen_64) - sizeof(struct fileio_hdr), &w);
				idx += w;
				memcpy(&pixels, screen_64->data, sizeof(pixels));
				
				ui_delay_wait(); // wait for previous delay to complete
				
				GPIOE_PDOR &= 0xDFFFFFFF;
			}
			else if (hdr->opcode == FILEIO_OPCODE_DELAY) {				
				fresult = f_read(&fil, fileio_buf + sizeof(struct fileio_hdr), 
					sizeof(struct fileio_opcode_delay) - sizeof(struct fileio_hdr), &w);
				idx += w;
				//
				// should create it's own function
				//
				//GPIOE_PDOR |= 0x20000000; // to turn LED on to turn on
				//ui_delay(delay->delay); // somehow delay->delay is always 0xDD, not sure where this comes from
																	// it didn't change when b2m.c changed from 200 to 63
				//ui_delay(50);
				
				ui_delay_set(40); // 40 gives you dead on 50ms according to scope
				GPIOE_PDOR |= 0x20000000;
				
				//GPIOE_PDOR &= 0xDFFFFFFF;
			}
		}
	}
	
  ui_send_str("done!\r\n", strlen("done!\r\n"));

	f_close(&fil);
	
}
