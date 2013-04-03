#include "types.h"
#include "derivative.h" /* include peripheral declarations */
#include "user_config.h"
#include "RealTimerCounter.h"
#include "Wdt_kinetis.h"
#include "hidef.h"
#include "usb_dci_kinetis.h"
#include "usb_dciapi.h"
#include "user_config.h"
#include "usb_cdc.h"        /* USB CDC Class Header File */
#include "virtual_com.h"
#include "i2c.h"
#include "led.h"
#include "ui.h"
#include "spi.h"
#include "ff.h"
#include "diskio.h"


#if MAX_TIMER_OBJECTS
        extern uint_8 TimerQInitialize(uint_8 ControllerId);
#endif

#define BSP_CLOCK_SRC                   (8000000ul)             // crystal, oscillator freq.
#define BSP_REF_CLOCK_SRC               (2000000ul)                     // must be 2-4MHz


#define BSP_CORE_DIV                    (1)
#define BSP_BUS_DIV                     (1)
#define BSP_FLEXBUS_DIV                 (1)
#define BSP_FLASH_DIV                   (2)

        // BSP_CLOCK_MUL from interval 24 - 55
#define BSP_CLOCK_MUL                   (24)    // 48MHz

#define BSP_REF_CLOCK_DIV               (BSP_CLOCK_SRC / BSP_REF_CLOCK_SRC)

#define BSP_CLOCK                       (BSP_REF_CLOCK_SRC * BSP_CLOCK_MUL)
#define BSP_CORE_CLOCK                  (BSP_CLOCK / BSP_CORE_DIV)          // CORE CLK, max 100MHz
#define BSP_SYSTEM_CLOCK                (BSP_CORE_CLOCK)                    // SYSTEM CLK, max 100MHz
#define BSP_BUS_CLOCK                   (BSP_CLOCK / BSP_BUS_DIV)       // max 50MHz
#define BSP_FLEXBUS_CLOCK               (BSP_CLOCK / BSP_FLEXBUS_DIV)
#define BSP_FLASH_CLOCK                 (BSP_CLOCK / BSP_FLASH_DIV)     // max 25MHz

//
// function prototypes
//

static void GPIO_Init(void);
static void USB_Init(uint_8 controller_ID);

//
// imported variables
//
 
volatile extern uint_32	ui_demo_mode; // from ui.c
volatile extern uint_32 usb_io_led;
volatile extern uint_32 ui_delay_counter; 

//
// global variables
//

uint_32 systick = 0;
FATFS g_sFatFs;

//
// interrupt handlers
//


void SysTick_Handler (void) {
	//
	// this is the systick interrupt handler, it should
	// execute 1/LED_SYSTICKS (4096) seconds
	static uint_32 diskproc = 0;
	static uint_32 t;
	led_systick_handler();
	
	systick++;
	
	//if (!(systick % UI_DEMO_RANDOM_RATE)) {
	//	ui_send_str(UI_HELP, strlen(UI_HELP));
	//}
	
	switch (ui_demo_mode) {
		case UI_DEMO_RANDOM:
			if (!(systick % UI_DEMO_RANDOM_RATE)) {
				led_random();
			}
			break;
	
		case UI_DEMO_NYAN:
			if (!(systick % UI_DEMO_NYAN_RATE)) {
				ui_demo_nyan();
			}
			break;
			
		default:
			break;
		}
	
		// flash LED when ther is a packet
		if (usb_io_led) {
			usb_io_led--;
			GPIOE_PDOR |= 0x20000000; // to turn LED on to turn on
		}
		else {
			GPIOE_PDOR &= 0xDFFFFFFF;
		}
		
		// run diskproc every 10ms at 1/4096s rate
		if (diskproc++ >= 39) {
				diskproc = 0;
				disk_timerproc();
		}
		
		// process ui delay
		t = ui_delay_counter;
		if (t) {
			ui_delay_counter = t - 1;
		}
}
 
//
// main loop
//

int main (void) {
		// init subsystems (SPI must come online before FatFS
    USB_Init(MAX3353);
    GPIO_Init();
		led_init(); // includes i2c
		spi1_init(); // must happen before FatFS		

	
		SysTick_Config(BSP_CLOCK/LED_SYSTICKS); // should be exported from system_MKL25Z4.c
																	  // 4096 is minimum for proper refresh rate without flicker
		
		f_mount(0, &g_sFatFs);
		

#if MAX_TIMER_OBJECTS
    (void)TimerQInitialize(0);
#endif
    
    /* Initialize the USB Application */
    vcom_main_init();
		
    
		for (;;) {
        //Watchdog_Reset();
        /* Call the application tasks */
        vcom_main_task();
				ui_process_string();
    }

    return 0;
}

static void GPIO_Init() {   


  //
	// Enable PORT D as output
	//
	
	/* Enable clock gating to PORTD */
  SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
	
	PORTD_PCR0 = PORT_PCR_MUX(1);
	PORTD_PCR0 |= PORT_PCR_DSE_MASK; // high drive strength
	PORTD_PCR1 = PORT_PCR_MUX(1);
	PORTD_PCR1 |= PORT_PCR_DSE_MASK;
	PORTD_PCR2 = PORT_PCR_MUX(1);
	PORTD_PCR2 |= PORT_PCR_DSE_MASK;
	PORTD_PCR3 = PORT_PCR_MUX(1);
	PORTD_PCR3 |= PORT_PCR_DSE_MASK;
	PORTD_PCR4 = PORT_PCR_MUX(1);
	PORTD_PCR4 |= PORT_PCR_DSE_MASK;
	PORTD_PCR5 = PORT_PCR_MUX(1);
	PORTD_PCR5 |= PORT_PCR_DSE_MASK;
	PORTD_PCR6 = PORT_PCR_MUX(1);
	PORTD_PCR6 |= PORT_PCR_DSE_MASK;
	PORTD_PCR7 = PORT_PCR_MUX(1);
	PORTD_PCR7 |= PORT_PCR_DSE_MASK;
	
	GPIOD_PDDR |= 0xFF;
	GPIOD_PDOR = 0xFE;
	
	/* Enable PTE0,1,2 and drive low to set enable LOW */
	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	PORTE_PCR0 = PORT_PCR_MUX(1);
	PORTE_PCR0 |= PORT_PCR_DSE_MASK; // high drive strength
	PORTE_PCR1 = PORT_PCR_MUX(1);
	PORTE_PCR1 |= PORT_PCR_DSE_MASK;
	PORTE_PCR2 = PORT_PCR_MUX(1);
	PORTE_PCR2 |= PORT_PCR_DSE_MASK;
	PORTE_PCR29 = PORT_PCR_MUX(1);
	PORTE_PCR29 |= PORT_PCR_DSE_MASK;
	PORTE_PCR30 = PORT_PCR_MUX(1);
	PORTE_PCR30 |= PORT_PCR_DSE_MASK;
	
	GPIOE_PDDR |= 0x60000007; // PTE29, 30 are LED 1,2
	GPIOE_PDOR =  0x400000F8; // |= 0x20000000 to turn on

#if defined(MCU_MKL25Z4)  
    /* Enable clock gating to PORTA, PORTB, PORTC, PORTD and PORTE */
    SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK 
              | SIM_SCGC5_PORTB_MASK 
              | SIM_SCGC5_PORTC_MASK 
              | SIM_SCGC5_PORTD_MASK 
              | SIM_SCGC5_PORTE_MASK);
#endif
}


static void USB_Init(uint_8 controller_ID) {
        if (controller_ID == MAX3353) {
                /* Enable USB-OTG IP clocking */
                SIM_SCGC4 |= (SIM_SCGC4_USBOTG_MASK);          

                /* Configure USB to be clocked from PLL */
                SIM_SOPT2  |= (SIM_SOPT2_USBSRC_MASK | SIM_SOPT2_PLLFLLSEL_MASK);

                /* Configure enable USB regulator for device */
                SIM_SOPT1 |= SIM_SOPT1_USBREGEN_MASK;

				NVIC_ICER |= (1<<24);    /* Clear any pending interrupts on USB */
				NVIC_ISER |= (1<<24);    /* Enable interrupts from USB module */                    

        } // MAX3353
        else if (controller_ID == ULPI) {
		   // not not our controller type
		}
}
