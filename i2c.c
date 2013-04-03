#include "types.h"
#include "derivative.h" /* include peripheral declarations */
#include "user_config.h"
#include "string.h"
#include "i2c.h"

static uint_8		i2c_initialize	= FALSE;
i2c_buf_t				i2c_buf;

void i2c_init (void) {
	/*
	 * this function initializes the I2C bus to use interrupts
	 */
	
	if (i2c_initialize == TRUE) {
		return; // already initialized
	}
	
	memset(&i2c_buf, 0x00, sizeof(i2c_buf));
	
	/* SIM_SCGC4: I2C0=1 */
  SIM_SCGC4 |= SIM_SCGC4_I2C0_MASK;                                                   
  /* I2C0_C1: IICEN=0,IICIE=0,MST=0,TX=0,TXAK=0,RSTA=0,WUEN=0,DMAEN=0 */
  I2C0_C1 = 0x00U;                     /* Clear control register */
  /* I2C0_FLT: SHEN=0,STOPF=1,STOPIE=0,FLT=0 */
  I2C0_FLT = I2C_FLT_STOPF_MASK;       /* Clear bus status interrupt flags */
  /* I2C0_S: TCF=0,IAAS=0,BUSY=0,ARBL=0,RAM=0,SRW=0,IICIF=1,RXAK=0 */
  I2C0_S = I2C_S_IICIF_MASK;           /* Clear interrupt flag */
  /* PORTB_PCR1: ISF=0,MUX=2 */
  PORTB_PCR1 = (uint32_t)((PORTB_PCR1 & (uint32_t)~(uint32_t)(
                PORT_PCR_ISF_MASK |
                PORT_PCR_MUX(0x05)
               )) | (uint32_t)(
                PORT_PCR_MUX(0x02)
               ));                                                  
  /* PORTB_PCR0: ISF=0,MUX=2 */
  PORTB_PCR0 = (uint32_t)((PORTB_PCR0 & (uint32_t)~(uint32_t)(
                PORT_PCR_ISF_MASK |
                PORT_PCR_MUX(0x05)
               )) | (uint32_t)(
                PORT_PCR_MUX(0x02)
               ));                                                  
  /* NVIC_IPR2: PRI_8=0x80 */
  NVIC_IPR2 = (uint32_t)((NVIC_IPR2 & (uint32_t)~(uint32_t)(
               NVIC_IP_PRI_8(0x7F)
              )) | (uint32_t)(
               NVIC_IP_PRI_8(0x80)
              ));                                                  
  /* NVIC_ISER: SETENA|=0x0100 */
  NVIC_ISER |= NVIC_ISER_SETENA(0x0100);                                                   
  /* I2C0_C2: GCAEN=0,ADEXT=0,HDRS=0,SBRC=0,RMEN=0,AD=0 */
  I2C0_C2 = 0x00U;                                                   
  /* I2C0_FLT: SHEN=0,STOPF=0,STOPIE=0,FLT=0 */
  I2C0_FLT = 0x00U;                    /* Set glitch filter register */
  /* I2C0_SMB: FACK=0,ALERTEN=0,SIICAEN=0,TCKSEL=0,SLTF=1,SHTF1=0,SHTF2=0,SHTF2IE=0 */
  I2C0_SMB = I2C_SMB_SLTF_MASK;                                                   
  /* I2C0_F: MULT=0,ICR=2 */
  //I2C0_F = I2C_F_ICR(0x02);            /* Set prescaler bits */
        /* I2C0_F: MULT=0,ICR=F = /68 p.706 of http://cache.freescale.com/files/32bit/doc/ref_manual/KL25P80M48SF0RM.pdf */
  I2C0_F = I2C_F_ICR(0x02); //I2C_F_ICR(0x0F);            /* Set prescaler bits */
	
	//I2C_PDD_EnableDevice(I2C0_BASE_PTR, PDD_ENABLE); /* Enable device */ #define PDD_ENABLE  1u
	/* Enable I2C device */
	I2C_C1_REG(I2C0_BASE_PTR) = (uint_8) ((I2C_C1_REG(I2C0_BASE_PTR) & ~I2C_C1_IICEN_MASK | 1u << I2C_C1_IICEN_SHIFT)); // TODO: fix this, I am pretty sure |= will work
	
  //I2C_PDD_EnableInterrupt(I2C0_BASE_PTR); /* Enable interrupt */
	I2C_C1_REG(I2C0_BASE_PTR) |= I2C_C1_IICIE_MASK;	
	i2c_initialize = TRUE;
}

void i2c1_deinit (void) {
	/*
	 * deinitialize I2C peripheral
	 */
	
	/* I2C0_C1: IICEN=0,IICIE=0,MST=0,TX=0,TXAK=0,RSTA=0,WUEN=0,DMAEN=0 */
  I2C0_C1 = 0x00U;                     /* Reset I2C Control register */
  /* SIM_SCGC4: I2C0=0 */
  SIM_SCGC4 &= (uint32_t)~(SIM_SCGC4_I2C0_MASK); 
	
}

void i2c_interrupt_process (i2c_buf_t *buf) { 
	/*
	 * this is the i2c interrupt handler, will be called from a wrapper function, 
	 * processor failexpert recommended using register for status, not sure if
	 * necessary. assuming device is always in master mode.
	 */
	
	uint8_t status = I2C_S_REG(I2C0_BASE_PTR);
	uint8_t errormask = 0x00;
	
	// clear interrupt flag
	I2C_S_REG(I2C0_BASE_PTR) = (uint_8)(status | ((I2C_S_IICIF_MASK & I2C_S_ARBL_MASK) & I2C_S_REG(I2C0_BASE_PTR)));
	
	if (I2C_PDD_GetMasterMode(I2C0_BASE_PTR) == I2C_PDD_MASTER_MODE) { /* Is device in master mode? */
    if (I2C_PDD_GetTransmitMode(I2C0_BASE_PTR) == I2C_PDD_TX_DIRECTION) { /* Is device in Tx mode? */
      if ((status & I2C_PDD_RX_ACKNOWLEDGE) != 0x00U){ /* NACK received? */
				
        I2C_PDD_SetMasterMode(I2C0_BASE_PTR, I2C_PDD_SLAVE_MODE); /* Switch device to slave mode (stop signal sent) */
        I2C_PDD_SetTransmitMode(I2C0_BASE_PTR, I2C_PDD_RX_DIRECTION); /* Switch to Rx mode */
				
        buf->tx_req = 0x00U; /* No character for sending */
        buf->rx_req = 0x00U; /* No character for reception */
        buf->flags &= ~(MASTER_IN_PROGRES); /* No character for sending or reception */
        buf->flags |= (ADDR_COMPLETE | REP_ADDR_COMPLETE); /* Set the flag */
        errormask |= LDD_I2C_MASTER_NACK; /* Set the Master Nack error mask */
      } 
			else {
        if ((buf->flags & ADDR_COMPLETE) != 0x00U) { /* If 10-bit addr has been completed */
          if (buf->tx_req != 0x00U) { /* Is any char. for transmitting? */
            buf->tx_req--;  /* Decrease number of chars for the transmit */
						
            I2C_PDD_WriteDataReg(I2C0_BASE_PTR, buf->tx_buf[buf->tx_idx++]); /* Send character */
						
          }
          else {
            if (buf->rx_req != 0x00U) { /* Is any char. for reception? */
              if ((buf->flags & REP_ADDR_COMPLETE) != 0x00U) { /* If repeated start and addr tx has been completed for 10-bit mode ?*/
                if (buf->rx_req == 0x01U) { /* If only one char to receive */
									
                  I2C_PDD_EnableTransmitAcknowledge(I2C0_BASE_PTR, PDD_DISABLE); /* then transmit ACK disable */
									
                } 
								else {
									
                  I2C_PDD_EnableTransmitAcknowledge(I2C0_BASE_PTR, PDD_ENABLE); /* else transmit ACK enable */
									
                }
								
                I2C_PDD_SetTransmitMode(I2C0_BASE_PTR, I2C_PDD_RX_DIRECTION); /* Switch to Rx mode */
                (void)I2C_PDD_ReadDataReg(I2C0_BASE_PTR); /* Dummy read character */
								
              } 
							else {                 /* Repeated address has not been completed for 10-bit addressing mode */
								
                I2C_PDD_RepeatStart(I2C0_BASE_PTR); /* Repeat start cycle generated */
								I2C_PDD_WriteDataReg(I2C0_BASE_PTR, (uint8_t)(buf->slave_addr_high | 0x01U)); /* Send slave address high byte*/
								
								buf->flags |= REP_ADDR_COMPLETE;
              }
            }
            else {
              buf->flags &= ~(MASTER_IN_PROGRES); /* Clear flag "busy" */
              if (buf->sendstop == TRUE) {
								
                I2C_PDD_SetMasterMode(I2C0_BASE_PTR, I2C_PDD_SLAVE_MODE); /* Switch device to slave mode (stop signal sent) */
                I2C_PDD_SetTransmitMode(I2C0_BASE_PTR, I2C_PDD_RX_DIRECTION); /* Switch to Rx mode */
								
              }
            }
          }
        } else {
          
					I2C_PDD_WriteDataReg(I2C0_BASE_PTR, buf->slave_addr_low); /* Send second part of the 10-bit addres */
										
          buf->flags |= (ADDR_COMPLETE); /* Address complete */
        }
      }
    }
    else {
      buf->rx_req--;        /* Decrease number of chars for the receive */
      if (buf->rx_req != 0x00U) { /* Is any char. for reception? */
        if (buf->rx_req == 0x01U) {
					
          I2C_PDD_EnableTransmitAcknowledge(I2C0_BASE_PTR, PDD_DISABLE); /* Transmit NACK */
					
        }
      } else {
        buf->flags &= ~(MASTER_IN_PROGRES); /* Clear flag "busy" */
				
        I2C_PDD_SetMasterMode(I2C0_BASE_PTR, I2C_PDD_SLAVE_MODE); /* If no, switch device to slave mode (stop signal sent) */
        I2C_PDD_EnableTransmitAcknowledge(I2C0_BASE_PTR, PDD_ENABLE); /* Transmit ACK */
				
      }
      buf->rx_buf[buf->rx_idx++] = I2C_PDD_ReadDataReg(I2C0_BASE_PTR); /* Receive character */
    }
  } else {
    if ((status & I2C_PDD_ARBIT_LOST) != 0x00U) { /* Arbitration lost? */
      buf->tx_req = 0x00U;  /* Any character is not for sent */
      buf->rx_req = 0x00U;  /* Any character is not for reception */
      buf->sendstop = TRUE; /* Set variable for sending stop condition (for master mode) */
      buf->flags &= ~(MASTER_IN_PROGRES); /* Any character is not for sent or reception*/
			
      I2C_PDD_SetTransmitMode(I2C0_BASE_PTR, I2C_PDD_RX_DIRECTION); /* Switch to Rx mode */
			
      errormask |= LDD_I2C_ARBIT_LOST; /* Set the ArbitLost error mask */
    }
  }
  if (errormask != 0x00) {            /* Is any error mask set? */
    buf->errors |= errormask; /* Update list of error mask value */
    // maybe do something here to handle error?
  }	
}


uint_8 i2c_slave_select (i2c_buf_t *buf, uint_8 addr_width, uint_32 addr) {
	/*
	 * this function selects the slave address of the i2c communication target,
	 * valid values for addr_width are either 7, 10 or 0, 0 indicates general call
	 */
	
	if ((buf->flags & MASTER_IN_PROGRES) != 0x00) {
		return(E_BUSY);
	}
	
	switch (addr_width) {
		case 0: // general call
			buf->flags &= ~(ADDR_7 | ADDR_10); /* Clear the 7-bit address flag and 10-bit address mode flag */
      buf->flags |= GENERAL_CALL; /* Set general call mode flag */
		break;
		
		case 7: // 7 bit addr
			if (addr > 0x7F) { // too many bits set
				return(E_BOUNDS);
			}
			buf->slave_addr_high = 0x00;
			buf->slave_addr_low = (uint8_t)((uint8_t)addr << 1); /* Set slave address */
      buf->flags &= ~(GENERAL_CALL | ADDR_10); /* Clear the general call flag and 10-bit address mode flag */
      buf->flags |= ADDR_7; /* Set 7-bit address mode flag */
		break;
		
		case 10: // 10 bit addr
			if (addr > 0x03FFU) { 
        return(E_BOUNDS);     
      }
      buf->slave_addr_low = (uint8_t)addr; /* Set slave address - low byte */
      buf->slave_addr_high = (uint8_t)((uint32_t)addr >> 7); /* Set slave address - high byte*/
      buf->slave_addr_high &= 0x06; /* Format address to 11110xx0 */
      buf->slave_addr_high |= 0xF0;
      buf->flags &= ~(GENERAL_CALL | ADDR_7); /* Clear the general call flag and 7-bit address mode flag */
      buf->flags |= ADDR_10; /* Set 10-bit address mode flag */
		break;
		
		default:
			return(E_BOUNDS);		
	}
	
	return(E_OK);	
}

uint_8 i2c_master_send_block (i2c_buf_t *buf, uint_8 *ptr, uint_32 length) {
	/*
	 * this function sends a block of data out of the i2c port as master, it also ensures that a
	 * stop condition is sent at the end
	 */
	
	if (!length) {
		return(E_OK); // nothing to do
	}
	if (length > I2C_BUF_MAX) {
		return(E_BOUNDS);
	}
	
	if ((I2C_PDD_GetBusStatus(I2C0_BASE_PTR) == I2C_PDD_BUS_BUSY) || /* Is the bus busy? */  \
			((buf->flags & MASTER_IN_PROGRES) != 0x00U) || (buf->tx_req != 0x00U)) {
		return(E_BUSY);
	}
	
	//
	// begin critical section
	//
	
	buf->flags |= MASTER_IN_PROGRES; /* Set flag "busy" */
	memcpy(buf->tx_buf, ptr, length);
	buf->rx_idx = 0;
	buf->tx_idx = 0;
	buf->rx_req = 0;
  buf->tx_req = length;       /* Set the counter of output bufer's content */
  buf->sendstop = TRUE;  /* Set generating stop condition */
  
	I2C_PDD_SetTransmitMode(I2C0_BASE_PTR, I2C_PDD_TX_DIRECTION); /* Set TX mode */
	
  if (I2C_PDD_GetMasterMode(I2C0_BASE_PTR) == I2C_PDD_MASTER_MODE) { /* Is device in master mode? */
    I2C_PDD_RepeatStart(I2C0_BASE_PTR); /* If yes then repeat start cycle generated */
  } 
	else {
    I2C_PDD_SetMasterMode(I2C0_BASE_PTR, I2C_PDD_MASTER_MODE); /* If no then start signal generated */
  }
  if ((buf->flags & ADDR_7) != 0x00) { /* Is 7-bit addressing set ? */
    buf->flags |= (ADDR_COMPLETE | REP_ADDR_COMPLETE); /* Only one byte of address will be sent 7-bit address mode*/
    I2C_PDD_WriteDataReg(I2C0_BASE_PTR, buf->slave_addr_low); /* Send slave address */
  } 
  else if ((buf->flags & ADDR_10) != 0x00) { /* Is 10-bit addressing set ? */
    buf->flags &= ~(ADDR_COMPLETE | REP_ADDR_COMPLETE); /* Second byte of address will be sent later */
    I2C_PDD_WriteDataReg(I2C0_BASE_PTR, buf->slave_addr_high); /* Send slave address - high byte*/
  } 
	else if ((buf->flags & GENERAL_CALL) != 0x00) { /* Is general call command required ? */
    buf->flags |= ADDR_COMPLETE; /* Only one byte of address will be sent in general call address mode*/
    I2C_PDD_WriteDataReg(I2C0_BASE_PTR, 0x00); /* Send general call address */
  }
	
	//
	// end critical section
	//
	
	return(E_OK);
}

uint_8 i2c_send_cmd (uint_8 *ptr, uint_32 length, uint_32 addr) {
	/*
	 * this function sends a complete command to addr, designed to work with 7bit addresses
	 * may want to add init function here since it checks every time 
	 */
	
	uint_8 e;
	
	if (i2c_initialize == FALSE) {
		return(E_BUSY);
	}
	
	if (i2c_buf.flags & MASTER_IN_PROGRES) {
		return(E_BUSY);
	}
	
	e = i2c_slave_select (&i2c_buf, 7, addr);
	
	if (e != E_OK) {
		return(e);
	}
	
	e = i2c_master_send_block (&i2c_buf, ptr, length);
	
	if (e != E_OK) {
		return(e);
	}
	
	// wait for the transaction to complete
	while (i2c_buf.flags & MASTER_IN_PROGRES); // gets stuck here
	
	return(E_OK);
}

void I2C0_IRQHandler (void) {
	i2c_interrupt_process (&i2c_buf);
}

