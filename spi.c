#include "types.h"
#include "spi.h"

static uint_8 spi1_initialized = FALSE;

void spi1_init (void) {
	/*
	 * this function initializes SPI to operate in non-interrupt mode
	 * clock is set to around 8MHz
	 */
	
	if (spi1_initialized == TRUE) {
		// already initialzed
		return;
	}
	
	/* SIM_SCGC4: SPI1=1 */
  SIM_SCGC4 |= SIM_SCGC4_SPI1_MASK;                                                   
  /* PORTB_PCR17: ISF=0,MUX=2 */
  PORTB_PCR17 = (uint32_t)((PORTB_PCR17 & (uint32_t)~(uint32_t)(
                 PORT_PCR_ISF_MASK |
                 PORT_PCR_MUX(0x05)
                )) | (uint32_t)(
                 PORT_PCR_MUX(0x02)
                ));        
  // enable pull-up on MISO in leu of resistor
	PORTB_PCR17 = (uint32_t)(PORTB_PCR17 | 0x3);
	
	
  /* PORTB_PCR16: ISF=0,MUX=2 */
  PORTB_PCR16 = (uint32_t)((PORTB_PCR16 & (uint32_t)~(uint32_t)(
                 PORT_PCR_ISF_MASK |
                 PORT_PCR_MUX(0x05)
                )) | (uint32_t)(
                 PORT_PCR_MUX(0x02)
                ));                                                  
  /* PORTB_PCR11: ISF=0,MUX=2 */
  PORTB_PCR11 = (uint32_t)((PORTB_PCR11 & (uint32_t)~(uint32_t)(
                 PORT_PCR_ISF_MASK |
                 PORT_PCR_MUX(0x05)
                )) | (uint32_t)(
                 PORT_PCR_MUX(0x02)
                ));                                                  
  /* PORTB_PCR10: ISF=0,MUX=2 */
	// we have manual SELECT control
  //PORTB_PCR10 = (uint32_t)((PORTB_PCR10 & (uint32_t)~(uint32_t)(
  //               PORT_PCR_ISF_MASK |
  //               PORT_PCR_MUX(0x05)
  //              )) | (uint32_t)(
  //               PORT_PCR_MUX(0x02)
  //              ));         
	PORTB_PCR10 = PORT_PCR_MUX(1); // set to gpio, high pull strength
	PORTB_PCR10 |= PORT_PCR_DSE_MASK;	
	GPIOB_PDOR |=  0x400; // set to high
	GPIOB_PDDR |= 0x400; // direction to output (pin 10, starting with 0)
	
  /* SPI1_C1: SPIE=0,SPE=0,SPTIE=0,MSTR=1,CPOL=0,CPHA=0,SSOE=1,LSBFE=0 */
  SPI1_C1 = (SPI_C1_MSTR_MASK | SPI_C1_SSOE_MASK); /* Set configuration register */
  /* SPI1_C2: SPMIE=0,SPLPIE=0,TXDMAE=0,MODFEN=1,BIDIROE=0,RXDMAE=0,SPISWAI=0,SPC0=0 */
  SPI1_C2 = SPI_C2_MODFEN_MASK;        /* Set configuration register */
  /* SPI1_BR: ??=0,SPPR=2,SPR=0 */
  SPI1_BR = SPI_BR_SPPR(0x02);         /* Set baud rate register */ // 8MHz
	//SPI1_BR = (SPI_BR_SPPR(0x02) | SPI_BR_SPR(0x03)); /* Set baud rate register */ // 1Mhz
  /* SPI1_C1: SPE=1 */
  SPI1_C1 |= SPI_C1_SPE_MASK;          /* Enable SPI module */
	
	
	
	spi1_initialized = TRUE;
}

void spi1_deinit (void) {
	/*
	 * disabled SPI1 peripheral
	 */
	SPI1_C1 = SPI_C1_CPHA_MASK;          /* Disable device */
  /* SIM_SCGC4: SPI1=0 */
  SIM_SCGC4 &= (uint32_t)~(uint32_t)(SIM_SCGC4_SPI1_MASK); 
}

void spi1_set_select (uint_8 s) {
	/*
	 * this function sets the SS pin (PTB10) low if s == 0, 
	 * and high otherwise
	 */
	if (s) {
		GPIOB_PDOR |=  0x400; // set to high
	}
	else {
		GPIOB_PDOR &= 0xFFFFFBFF;
	}
}

uint_8 spi1_byte_xmit (uint_8 tx, uint_8 *rx) {
	/*
	 * this function does a blocking transmission of a single byte
	 */
	
	if (spi1_initialized == FALSE) {
		spi1_init();
	}
	
	while ((SPI_S_REG(SPI1_BASE_PTR) & SPI_S_SPTEF_MASK) == 0x00); // wait for TX to become empty
	
	SPI_D_REG(SPI1_BASE_PTR) = tx;
	
	while ((SPI_S_REG(SPI1_BASE_PTR) & SPI_S_SPRF_MASK) == 0x00); // wait for RX
	*rx = SPI_D_REG(SPI1_BASE_PTR);
	
	return(E_OK);
}
