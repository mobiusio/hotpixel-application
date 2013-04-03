#ifndef _I2C_H_
#define _I2C_H_

#define I2C_BUF_MAX						16

typedef struct {
	uint_8		rx_buf[I2C_BUF_MAX]; // data buffer
	uint_8		tx_buf[I2C_BUF_MAX];
	volatile uint_32		rx_idx; // current rx byte index
	volatile uint_32		tx_idx; // current tx byte index
	volatile uint_32		rx_req; // number of rx bytes expected in this transaction
	volatile uint_32		tx_req; // number of tx bytes expected in this transaction
	volatile uint_32		flags; // general flags
	volatile uint_8		errors;
	volatile uint_8		sendstop; // indicate we want to send a stop condition
	volatile uint_8		slave_addr_high;
	volatile uint_8		slave_addr_low;
	} i2c_buf_t;

void i2c_init (void);
void i2c_deinit (void);
void i2c_interrupt_process (i2c_buf_t *buf);
uint_8 i2c_slave_select (i2c_buf_t *buf, uint_8 addr_width, uint_32 addr);
uint_8 i2c_master_send_block (i2c_buf_t *buf, uint_8 *ptr, uint_32 length);
uint_8 i2c_send_cmd (uint_8 *ptr, uint_32 length, uint_32 addr);


//
// begin the macro superfun time
//

/*
** ===================================================================
** I2C device types and constants
** ===================================================================
*/

#define LDD_I2C_SDA_PIN                        0x01u /* SDA pin mask */
#define LDD_I2C_SCL_PIN                        0x02u /* SCL pin  mask */

#define LDD_I2C_ON_MASTER_BLOCK_SENT           0x0001u /*  OnMasterBlockSent event mask */
#define LDD_I2C_ON_MASTER_BLOCK_RECEIVED       0x0002u /*  OnMasterBlockReceived event mask */
#define LDD_I2C_ON_SLAVE_BLOCK_SENT            0x0004u /*  OnSlaveBlockSent event mask */
#define LDD_I2C_ON_SLAVE_BLOCK_RECEIVED        0x0008u /*  OnSlaveBlockReceived event mask */
#define LDD_I2C_ON_SLAVE_TX_REQUEST            0x0010u /*  OnSlaveTxRequest event mask */
#define LDD_I2C_ON_SLAVE_RX_REQUEST            0x0020u /*  OnSlaveRxRequest event mask */
#define LDD_I2C_ON_ERROR                       0x0040u /*  OnError event mask */
#define LDD_I2C_ON_SLAVE_SM_BUS_CALL_ADDR      0x0080u /*  OnSlaveSMBusCallAddr event mask */
#define LDD_I2C_ON_SLAVE_SM_BUS_ALERT_RESPONSE 0x0100u /*  OnSlaveSMBusAlertResponse event mask */
#define LDD_I2C_ON_SLAVE_GENERAL_CALL_ADDR     0x0200u /*  OnSlaveGeneralCallAddr event mask */
#define LDD_I2C_ON_MASTER_BYTE_RECEIVED        0x0400u /*  OnMasterByteReceived event mask */
#define LDD_I2C_ON_SLAVE_BYTE_RECEIVED         0x0800u /*  OnMasterByteReceived event mask */

#define LDD_I2C_SLAVE_TX_UNDERRUN              0x0001u /*  SlaveTxUnderrun error mask */
#define LDD_I2C_SLAVE_RX_OVERRUN               0x0002u /*  SlaveRxOverrun error mask */
#define LDD_I2C_ARBIT_LOST                     0x0004u /*  ArbitLost error mask */
#define LDD_I2C_MASTER_NACK                    0x0008u /*  MasterNACK error mask */
#define LDD_I2C_SCL_LOW_TIMEOUT                0x0010u /*  SCLLowTimeout error mask */
#define LDD_I2C_SDA_LOW_TIMEOUT                0x0020u /*  SDALowTimeout error mask */
#define LDD_I2C_SLAVE_NACK                     0x0040u /*  SlaveNACK error mask */

#define MASTER_IN_PROGRES       0x01U  /* Communication is in progress (Master) */
#define ADDR_COMPLETE           0x02U  /* 10-bit address transmission complete   */
#define REP_ADDR_COMPLETE       0x04U  /* repeated address transmission complete */
#define GENERAL_CALL            0x08U  /* General call flag */
#define ADDR_10                 0x10U  /* 10-bit addr flag */
#define ADDR_7                  0x20U  /* 7-bit addr flag */

/* ----------------------------------------------------------------------------
   -- Method symbol definitions
   ---------------------------------------------------------------------------- */

/* Status flags constants (for ReadStatusReg, GetInterruptFlags,
   ClearInterruptFlags macros). */
#define I2C_PDD_RX_ACKNOWLEDGE I2C_S_RXAK_MASK   /**< Receive acknowledge. */
#define I2C_PDD_INTERRUPT_FLAG I2C_S_IICIF_MASK  /**< Interrupt flag. */
#define I2C_PDD_SLAVE_TRANSMIT I2C_S_SRW_MASK    /**< Slave read/write request. */
#define I2C_PDD_RANGE_ADDRESS_MATCH I2C_S_RAM_MASK /**< Range address match. */
#define I2C_PDD_ARBIT_LOST I2C_S_ARBL_MASK       /**< Arbitration lost. */
#define I2C_PDD_BUS_IS_BUSY I2C_S_BUSY_MASK      /**< Bus busy - set when a START signal is detected. */
#define I2C_PDD_ADDRESSED_AS_SLAVE I2C_S_IAAS_MASK /**< Addressed as a slave. */
#define I2C_PDD_TX_COMPLETE I2C_S_TCF_MASK       /**< Transfer complete flag. */

/* Status flags constants. */
#define I2C_PDD_BUS_STOP_FLAG I2C_FLT_STOPF_MASK /**< Stop detected on I2C bus */
#define I2C_PDD_BUS_START_FLAG I2C_FLT_STARTF_MASK /**< Start detected on I2C bus */

/* SCL timeout flags constants (for GetSCLTimeoutInterruptFlags,
   ClearSCLTimeoutInterruptFlags macros). */
#define I2C_PDD_SCL_LOW_TIMEOUT I2C_SMB_SLTF_MASK /**< SCL low timeout flag. */
#define I2C_PDD_SCL_HI_AND_SDA_HI_TIMEOUT I2C_SMB_SHTF1_MASK /**< SCL high timeout flag - sets when SCL and SDA are held high more than clock × LoValue / 512. */
#define I2C_PDD_SCL_HI_AND_SDA_LOW_TIMEOUT I2C_SMB_SHTF2_MASK /**< SCL high timeout flag - sets when SCL is held high and SDA is held low more than clock × LoValue/512. */

/* Frequency multiplier constants (for SetFrequencyMultiplier macro). */
#define I2C_PDD_FREQUENCY_MUL_1 0U               /**< Multiplier factor = 1 */
#define I2C_PDD_FREQUENCY_MUL_2 0x1U             /**< Multiplier factor = 2 */
#define I2C_PDD_FREQUENCY_MUL_4 0x2U             /**< Multiplier factor = 4 */

/* Master mode constants (for MasterMode, GetMasterMode macros). */
#define I2C_PDD_MASTER_MODE 0x20U                /**< Master mode. */
#define I2C_PDD_SLAVE_MODE 0U                    /**< Slave mode. */

/* Transmit mode constants (for SetTransmitMode, GetTransmitMode macros). */
#define I2C_PDD_TX_DIRECTION 0x10U               /**< SDA pin set as output. */
#define I2C_PDD_RX_DIRECTION 0U                  /**< SDA pin set as input. */

/* BUS status constants (for GetBusStatus macro). */
#define I2C_PDD_BUS_IDLE 0U                      /**< Bus is idle. */
#define I2C_PDD_BUS_BUSY 0x20U                   /**< Bus is busy. */

/* Fast SMBus Acknowledge constants (for GetFastSmBusAcknowledge macro). */
#define I2C_PDD_ACK_FOLLOWING_RX_DATA 0U         /**< ACK or NACK is sent on the following receiving data byte. */
#define I2C_PDD_ACK_AFTER_RX_DATA 0x80U          /**< ACK or NACK is sent after receiving a data byte. */

/* Clock source constants (for SetSCLTimeoutBusClockSource macro). */
#define I2C_PDD_BUS_CLOCK 0x10U                  /**< Bus clock frequency */
#define I2C_PDD_BUS_CLOCK_DIV64 0U               /**< Bus clock / 64 frequency */


#define PE_LDD_COMPONENT_I2C2_ID                 0x02U


#define PDD_DISABLE 0u
#define PDD_ENABLE  1u
/**
 * Enables/disables I2C device.
 * @param peripheralBase Peripheral base address.
 * @param State Requested state of device.
 */
#define I2C_PDD_EnableDevice(peripheralBase, State) ( \
    I2C_C1_REG(peripheralBase) = \
     (uint8_t)(( \
      (uint8_t)(I2C_C1_REG(peripheralBase) & (uint8_t)(~(uint8_t)I2C_C1_IICEN_MASK))) | ( \
      (uint8_t)((uint8_t)(State) << I2C_C1_IICEN_SHIFT))) \
  )


/**
 * Enables the I2C interrupt.
 * @param peripheralBase Peripheral base address.
 */
#define I2C_PDD_EnableInterrupt(peripheralBase) ( \
    I2C_C1_REG(peripheralBase) |= \
     I2C_C1_IICIE_MASK \
  )

/* ----------------------------------------------------------------------------
   -- DisableInterrupt
   ---------------------------------------------------------------------------- */

/**
 * Disables the I2C interrupt.
 * @param peripheralBase Peripheral base address.
 */
#define I2C_PDD_DisableInterrupt(peripheralBase) ( \
    I2C_C1_REG(peripheralBase) &= \
     (uint8_t)(~(uint8_t)I2C_C1_IICIE_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetMasterMode
   ---------------------------------------------------------------------------- */

/**
 * Puts the I2C to the master or slave mode.
 * @param peripheralBase Peripheral base address.
 * @param MasterMode I2C mode value. The user should use one from the enumerated
 *        values.
 */
#define I2C_PDD_SetMasterMode(peripheralBase, MasterMode) ( \
    I2C_C1_REG(peripheralBase) = \
     (uint16_t)(( \
      (uint16_t)(I2C_C1_REG(peripheralBase) & (uint16_t)(~(uint16_t)I2C_C1_MST_MASK))) | ( \
      (uint16_t)(MasterMode))) \
  )

/* ----------------------------------------------------------------------------
   -- GetMasterMode
   ---------------------------------------------------------------------------- */

/**
 * Returns the current operating mode.
 * @param peripheralBase Peripheral base address.
 */
#define I2C_PDD_GetMasterMode(peripheralBase) ( \
    (uint16_t)(I2C_C1_REG(peripheralBase) & I2C_C1_MST_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetTransmitMode
   ---------------------------------------------------------------------------- */

/**
 * Sets data pin to the output or input direction.
 * @param peripheralBase Peripheral base address.
 * @param TransmitMode Direction I2C pins. The user should use one from the
 *        enumerated values.
 */
#define I2C_PDD_SetTransmitMode(peripheralBase, TransmitMode) ( \
    I2C_C1_REG(peripheralBase) = \
     (uint16_t)(( \
      (uint16_t)(I2C_C1_REG(peripheralBase) & (uint16_t)(~(uint16_t)I2C_C1_TX_MASK))) | ( \
      (uint16_t)(TransmitMode))) \
  )

/* ----------------------------------------------------------------------------
   -- GetTransmitMode
   ---------------------------------------------------------------------------- */

/**
 * Returns the current direction mode.
 * @param peripheralBase Peripheral base address.
 */
#define I2C_PDD_GetTransmitMode(peripheralBase) ( \
    (uint16_t)(I2C_C1_REG(peripheralBase) & I2C_C1_TX_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableTransmitAcknowledge
   ---------------------------------------------------------------------------- */

/**
 * Enables/disables an acknowledge signal to be sent to the bus at the ninth
 * clock bit after receiving one byte of data.
 * @param peripheralBase Peripheral base address.
 * @param State Requested state of acknowledge signal.
 */
#define I2C_PDD_EnableTransmitAcknowledge(peripheralBase, State) ( \
    ((State) == PDD_ENABLE) ? ( \
      I2C_C1_REG(peripheralBase) &= \
       (uint16_t)(~(uint16_t)I2C_C1_TXAK_MASK)) : ( \
      I2C_C1_REG(peripheralBase) |= \
       (uint16_t)((uint16_t)0x1U << I2C_C1_TXAK_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- RepeatStart
   ---------------------------------------------------------------------------- */

/**
 * Generates a repeated START condition.
 * @param peripheralBase Peripheral base address.
 */
#define I2C_PDD_RepeatStart(peripheralBase) ( \
    I2C_C1_REG(peripheralBase) |= \
     I2C_C1_RSTA_MASK \
  )

/* ----------------------------------------------------------------------------
   -- EnableWakeUp
   ---------------------------------------------------------------------------- */

/**
 * Enables or disables the wakeup function in stop3 mode.
 * @param peripheralBase Peripheral base address.
 * @param State Requested state of wakeup function.
 */
#define I2C_PDD_EnableWakeUp(peripheralBase, State) ( \
    I2C_C1_REG(peripheralBase) = \
     (uint16_t)(( \
      (uint16_t)(I2C_C1_REG(peripheralBase) & (uint16_t)(~(uint16_t)I2C_C1_WUEN_MASK))) | ( \
      (uint16_t)((uint16_t)(State) << I2C_C1_WUEN_SHIFT))) \
  )

/* ----------------------------------------------------------------------------
   -- EnableDmaRequest
   ---------------------------------------------------------------------------- */

/**
 * Enables or disables the DMA transfer.
 * @param peripheralBase Peripheral base address.
 * @param State Requested state of DMA function.
 */
#define I2C_PDD_EnableDmaRequest(peripheralBase, State) ( \
    I2C_C1_REG(peripheralBase) = \
     (uint16_t)(( \
      (uint16_t)(I2C_C1_REG(peripheralBase) & (uint16_t)(~(uint16_t)I2C_C1_DMAEN_MASK))) | ( \
      (uint16_t)(State))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadControl1Reg
   ---------------------------------------------------------------------------- */

/**
 * Reads control 1 register.
 * @param peripheralBase Peripheral base address.
 */
#define I2C_PDD_ReadControl1Reg(peripheralBase) ( \
    I2C_C1_REG(peripheralBase) \
  )

/* ----------------------------------------------------------------------------
   -- WriteControl1Reg
   ---------------------------------------------------------------------------- */

/**
 * Writes new value specified by the Value parameter into control 1 register.
 * @param peripheralBase Peripheral base address.
 * @param Value Value to be written to the control 1 register.
 */
#define I2C_PDD_WriteControl1Reg(peripheralBase, Value) ( \
    I2C_C1_REG(peripheralBase) = \
     (uint16_t)(Value) \
  )

/* ----------------------------------------------------------------------------
   -- ReadStatusReg
   ---------------------------------------------------------------------------- */

/**
 * Returns the value of the status register.
 * @param peripheralBase Peripheral base address.
 * @return Use constants from group "Status flags constants (for ReadStatusReg,
 *         GetInterruptFlags, ClearInterruptFlags macros)." for processing return
 *         value.
 */
#define I2C_PDD_ReadStatusReg(peripheralBase) ( \
    I2C_S_REG(peripheralBase) \
  )

/* ----------------------------------------------------------------------------
   -- GetBusStatus
   ---------------------------------------------------------------------------- */

/**
 * Returns status of the BUS.
 * @param peripheralBase Peripheral base address.
 */
#define I2C_PDD_GetBusStatus(peripheralBase) ( \
    (uint16_t)(I2C_S_REG(peripheralBase) & I2C_S_BUSY_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetInterruptFlags
   ---------------------------------------------------------------------------- */

/**
 * Returns a value that represents a mask of active (pending) interrupts.
 * @param peripheralBase Peripheral base address.
 * @return Use constants from group "Status flags constants (for ReadStatusReg,
 *         GetInterruptFlags, ClearInterruptFlags macros)." for processing return
 *         value.
 */
#define I2C_PDD_GetInterruptFlags(peripheralBase) ( \
    (uint16_t)(( \
     I2C_S_REG(peripheralBase)) & ( \
     (uint16_t)(( \
      I2C_S_TCF_MASK) | (( \
      I2C_S_IICIF_MASK) | (( \
      I2C_S_RAM_MASK) | (( \
      I2C_S_ARBL_MASK) | ( \
      I2C_S_IAAS_MASK))))))) \
  )

/* ----------------------------------------------------------------------------
   -- ClearInterruptFlags
   ---------------------------------------------------------------------------- */

/**
 * Clears interrupt flags of interrupts specified by Mask.
 * @param peripheralBase Peripheral base address.
 * @param Mask Mask of interrupt clear requests. Use constants from group
 *        "Status flags constants (for ReadStatusReg, GetInterruptFlags,
 *        ClearInterruptFlags macros).".
 */
#define I2C_PDD_ClearInterruptFlags(peripheralBase, Mask) ( \
    I2C_S_REG(peripheralBase) = \
     (uint16_t)(( \
      (uint16_t)(( \
       I2C_S_REG(peripheralBase)) & (( \
       (uint16_t)(~(uint16_t)I2C_S_IICIF_MASK)) & ( \
       (uint16_t)(~(uint16_t)I2C_S_ARBL_MASK))))) | ( \
      (uint16_t)(Mask))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteStatusReg
   ---------------------------------------------------------------------------- */

/**
 * Writes new value specified by the Value parameter into status register.
 * @param peripheralBase Peripheral base address.
 * @param Value Value to be written to the status register.
 */
#define I2C_PDD_WriteStatusReg(peripheralBase, Value) ( \
    I2C_S_REG(peripheralBase) = \
     (uint16_t)(Value) \
  )

/* ----------------------------------------------------------------------------
   -- ReadDataReg
   ---------------------------------------------------------------------------- */

/**
 * Returns the content of the data register.
 * @param peripheralBase Peripheral base address.
 */
#define I2C_PDD_ReadDataReg(peripheralBase) ( \
    I2C_D_REG(peripheralBase) \
  )

/* ----------------------------------------------------------------------------
   -- WriteDataReg
   ---------------------------------------------------------------------------- */

/**
 * Writes data to the data register.
 * @param peripheralBase Peripheral base address.
 * @param Data Data value.
 */
#define I2C_PDD_WriteDataReg(peripheralBase, Data) ( \
    I2C_D_REG(peripheralBase) = \
     (uint16_t)(Data) \
  )

/* ----------------------------------------------------------------------------
   -- EnableGeneralCallAddress
   ---------------------------------------------------------------------------- */

/**
 * Enables or disables general call address.
 * @param peripheralBase Peripheral base address.
 * @param State Requested state of general call address function.
 */
#define I2C_PDD_EnableGeneralCallAddress(peripheralBase, State) ( \
    I2C_C2_REG(peripheralBase) = \
     (uint16_t)(( \
      (uint16_t)(I2C_C2_REG(peripheralBase) & (uint16_t)(~(uint16_t)I2C_C2_GCAEN_MASK))) | ( \
      (uint16_t)((uint16_t)(State) << I2C_C2_GCAEN_SHIFT))) \
  )

/* ----------------------------------------------------------------------------
   -- EnableAddressExtension
   ---------------------------------------------------------------------------- */

/**
 * Enables or disables extension address.
 * @param peripheralBase Peripheral base address.
 * @param State Requested state of extension address.
 */
#define I2C_PDD_EnableAddressExtension(peripheralBase, State) ( \
    I2C_C2_REG(peripheralBase) = \
     (uint16_t)(( \
      (uint16_t)(I2C_C2_REG(peripheralBase) & (uint16_t)(~(uint16_t)I2C_C2_ADEXT_MASK))) | ( \
      (uint16_t)((uint16_t)(State) << I2C_C2_ADEXT_SHIFT))) \
  )

/* ----------------------------------------------------------------------------
   -- SetPadsNormalDriveMode
   ---------------------------------------------------------------------------- */

/**
 * Sets control the drive capability of the I2C pads to normal drive mode.
 * @param peripheralBase Peripheral base address.
 */
#define I2C_PDD_SetPadsNormalDriveMode(peripheralBase) ( \
    I2C_C2_REG(peripheralBase) &= \
     (uint16_t)(~(uint16_t)I2C_C2_HDRS_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPadsHighDriveMode
   ---------------------------------------------------------------------------- */

/**
 * Sets control the drive capability of the I2C pads to high drive mode.
 * @param peripheralBase Peripheral base address.
 */
#define I2C_PDD_SetPadsHighDriveMode(peripheralBase) ( \
    I2C_C2_REG(peripheralBase) |= \
     I2C_C2_HDRS_MASK \
  )

/* ----------------------------------------------------------------------------
   -- EnableSlaveBaudControlByMaster
   ---------------------------------------------------------------------------- */

/**
 * Enables or disables slave baud rate control by master device.
 * @param peripheralBase Peripheral base address.
 * @param State Requested state of slave baud rate control.
 */
#define I2C_PDD_EnableSlaveBaudControlByMaster(peripheralBase, State) ( \
    I2C_C2_REG(peripheralBase) = \
     (uint16_t)(( \
      (uint16_t)(I2C_C2_REG(peripheralBase) & (uint16_t)(~(uint16_t)I2C_C2_SBRC_MASK))) | ( \
      (uint16_t)((uint16_t)(State) << I2C_C2_SBRC_SHIFT))) \
  )

/* ----------------------------------------------------------------------------
   -- EnableRangeAddressMatch
   ---------------------------------------------------------------------------- */

/**
 * Enables or disables range address matching.
 * @param peripheralBase Peripheral base address.
 * @param State Requested state of range address matching function.
 */
#define I2C_PDD_EnableRangeAddressMatch(peripheralBase, State) ( \
    I2C_C2_REG(peripheralBase) = \
     (uint16_t)(( \
      (uint16_t)(I2C_C2_REG(peripheralBase) & (uint16_t)(~(uint16_t)I2C_C2_RMEN_MASK))) | ( \
      (uint16_t)((uint16_t)(State) << I2C_C2_RMEN_SHIFT))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadControl2Reg
   ---------------------------------------------------------------------------- */

/**
 * Reads control 2 register.
 * @param peripheralBase Peripheral base address.
 */
#define I2C_PDD_ReadControl2Reg(peripheralBase) ( \
    I2C_C2_REG(peripheralBase) \
  )

/* ----------------------------------------------------------------------------
   -- WriteControl2Reg
   ---------------------------------------------------------------------------- */

/**
 * Writes new value specified by the Value parameter into control 2 register.
 * @param peripheralBase Peripheral base address.
 * @param Value Value to be written to the control 2 register.
 */
#define I2C_PDD_WriteControl2Reg(peripheralBase, Value) ( \
    I2C_C2_REG(peripheralBase) = \
     (uint16_t)(Value) \
  )

/* ----------------------------------------------------------------------------
   -- SetInputGlitchFilter
   ---------------------------------------------------------------------------- */

/**
 * Sets the programming controls for the width of glitch (in terms of bus clock
 * cycles) the filter must absorb.
 * @param peripheralBase Peripheral base address.
 * @param FilterFactorValue Input glitch filter value[0..15].
 */
#define I2C_PDD_SetInputGlitchFilter(peripheralBase, FilterFactorValue) ( \
    I2C_FLT_REG(peripheralBase) = \
     (uint16_t)(( \
      (uint16_t)(( \
       I2C_FLT_REG(peripheralBase)) & (( \
       (uint16_t)(~(uint16_t)I2C_FLT_FLT_MASK)) & (( \
       (uint16_t)(~(uint16_t)I2C_FLT_STARTF_MASK)) & ( \
       (uint16_t)(~(uint16_t)I2C_FLT_STOPF_MASK)))))) | ( \
      (uint16_t)(FilterFactorValue))) \
  )

/* ----------------------------------------------------------------------------
   -- EnableStopHoldOffMode
   ---------------------------------------------------------------------------- */

/**
 * Enables/disables stop mode holdoff.
 * @param peripheralBase Peripheral base address.
 * @param State Requested state of stop mode holdoff.
 */
#define I2C_PDD_EnableStopHoldOffMode(peripheralBase, State) ( \
    I2C_FLT_REG(peripheralBase) = \
     (uint16_t)(( \
      (uint16_t)(( \
       I2C_FLT_REG(peripheralBase)) & (( \
       (uint16_t)(~(uint16_t)I2C_FLT_SHEN_MASK)) & (( \
       (uint16_t)(~(uint16_t)I2C_FLT_STARTF_MASK)) & ( \
       (uint16_t)(~(uint16_t)I2C_FLT_STOPF_MASK)))))) | ( \
      (uint16_t)((uint16_t)(State) << I2C_FLT_SHEN_SHIFT))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadBusStatusFlags
   ---------------------------------------------------------------------------- */

/**
 * Returns the value of the bus status flags.
 * @param peripheralBase Peripheral base address.
 * @return Use constants from group "Status flags constants." for processing
 *         return value.
 */
#define I2C_PDD_ReadBusStatusFlags(peripheralBase) ( \
    I2C_FLT_REG(peripheralBase) \
  )

/* ----------------------------------------------------------------------------
   -- ClearBusStatusInterruptFlags
   ---------------------------------------------------------------------------- */

/**
 * Clears bus status interrupt flags of interrupts specified by Mask.
 * @param peripheralBase Peripheral base address.
 * @param Mask Mask of interrupt clear requests. Use constants from group
 *        "Status flags constants.".
 */
#define I2C_PDD_ClearBusStatusInterruptFlags(peripheralBase, Mask) ( \
    I2C_FLT_REG(peripheralBase) = \
     (uint16_t)(( \
      (uint16_t)(( \
       I2C_FLT_REG(peripheralBase)) & ( \
       (uint16_t)(~(uint16_t)(I2C_FLT_STOPF_MASK | I2C_FLT_STARTF_MASK))))) | ( \
      (uint16_t)(Mask))) \
  )

/* ----------------------------------------------------------------------------
   -- EnableBusStopOrStartInterrupt
   ---------------------------------------------------------------------------- */

/**
 * Enables the bus stop or start interrupt.
 * @param peripheralBase Peripheral base address.
 */
#define I2C_PDD_EnableBusStopOrStartInterrupt(peripheralBase) ( \
    I2C_FLT_REG(peripheralBase) = \
     (uint16_t)(( \
      (uint16_t)(I2C_FLT_REG(peripheralBase) | I2C_FLT_SSIE_MASK)) & (( \
      (uint16_t)(~(uint16_t)I2C_FLT_STARTF_MASK)) & ( \
      (uint16_t)(~(uint16_t)I2C_FLT_STOPF_MASK)))) \
  )

/* ----------------------------------------------------------------------------
   -- DisableBusStopOrStartInterrupt
   ---------------------------------------------------------------------------- */

/**
 * Disables the bus stop or start interrupt.
 * @param peripheralBase Peripheral base address.
 */
#define I2C_PDD_DisableBusStopOrStartInterrupt(peripheralBase) ( \
    I2C_FLT_REG(peripheralBase) &= \
     (uint16_t)(( \
      (uint16_t)(~(uint16_t)I2C_FLT_SSIE_MASK)) & (( \
      (uint16_t)(~(uint16_t)I2C_FLT_STARTF_MASK)) & ( \
      (uint16_t)(~(uint16_t)I2C_FLT_STOPF_MASK)))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadInputGlitchFilterReg
   ---------------------------------------------------------------------------- */

/**
 * Reads input glitch filter register.
 * @param peripheralBase Peripheral base address.
 */
#define I2C_PDD_ReadInputGlitchFilterReg(peripheralBase) ( \
    I2C_FLT_REG(peripheralBase) \
  )

/* ----------------------------------------------------------------------------
   -- WriteInputGlitchFilterReg
   ---------------------------------------------------------------------------- */

/**
 * Writes new value specified by the Value parameter into input glitch filter
 * register.
 * @param peripheralBase Peripheral base address.
 * @param Value Value to be written to the input glitch filter register.
 */
#define I2C_PDD_WriteInputGlitchFilterReg(peripheralBase, Value) ( \
    I2C_FLT_REG(peripheralBase) = \
     (uint16_t)(Value) \
  )

/* ----------------------------------------------------------------------------
   -- SetRangeAddress
   ---------------------------------------------------------------------------- */

/**
 * Sets the range slave address.
 * @param peripheralBase Peripheral base address.
 * @param RangeAddressValue Range Address value[0..127].
 */
#define I2C_PDD_SetRangeAddress(peripheralBase, RangeAddressValue) ( \
    I2C_RA_REG(peripheralBase) = \
     (uint16_t)(( \
      (uint16_t)(I2C_RA_REG(peripheralBase) & (uint16_t)(~(uint16_t)I2C_RA_RAD_MASK))) | ( \
      (uint16_t)((uint16_t)(RangeAddressValue) << I2C_RA_RAD_SHIFT))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadRangeAddressReg
   ---------------------------------------------------------------------------- */

/**
 * Reads value of range address register.
 * @param peripheralBase Peripheral base address.
 */
#define I2C_PDD_ReadRangeAddressReg(peripheralBase) ( \
    I2C_RA_REG(peripheralBase) \
  )

/* ----------------------------------------------------------------------------
   -- WriteRangeAddressReg
   ---------------------------------------------------------------------------- */

/**
 * Writes new value specified by the Value parameter into range address register.
 * @param peripheralBase Peripheral base address.
 * @param Value Value to be written to the range address register.
 */
#define I2C_PDD_WriteRangeAddressReg(peripheralBase, Value) ( \
    I2C_RA_REG(peripheralBase) = \
     (uint16_t)(Value) \
  )

/* ----------------------------------------------------------------------------
   -- EnableFastSmBusNackAck
   ---------------------------------------------------------------------------- */

/**
 * Enables or disables fast SMBus NACK/ACK.
 * @param peripheralBase Peripheral base address.
 * @param State Parameter specifying if SMBus alert response will be enabled or
 *        disabled.
 */
#define I2C_PDD_EnableFastSmBusNackAck(peripheralBase, State) ( \
    I2C_SMB_REG(peripheralBase) = \
     (uint16_t)(( \
      (uint16_t)(( \
       I2C_SMB_REG(peripheralBase)) & (( \
       (uint16_t)(~(uint16_t)I2C_SMB_FACK_MASK)) & (( \
       (uint16_t)(~(uint16_t)I2C_SMB_SHTF2_MASK)) & ( \
       (uint16_t)(~(uint16_t)I2C_SMB_SLTF_MASK)))))) | ( \
      (uint16_t)((uint16_t)(State) << I2C_SMB_FACK_SHIFT))) \
  )

/* ----------------------------------------------------------------------------
   -- GetFastSmBusAcknowledge
   ---------------------------------------------------------------------------- */

/**
 * Fast NACK/ACK enable bit.
 * @param peripheralBase Peripheral base address.
 */
#define I2C_PDD_GetFastSmBusAcknowledge(peripheralBase) ( \
    (uint16_t)(I2C_SMB_REG(peripheralBase) & I2C_SMB_FACK_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableSmBusAlertResponseAddress
   ---------------------------------------------------------------------------- */

/**
 * Enables or disables SMBus alert response address.
 * @param peripheralBase Peripheral base address.
 * @param State Parameter specifying if SMBus alert response will be enabled or
 *        disabled.
 */
#define I2C_PDD_EnableSmBusAlertResponseAddress(peripheralBase, State) ( \
    I2C_SMB_REG(peripheralBase) = \
     (uint16_t)(( \
      (uint16_t)(( \
       I2C_SMB_REG(peripheralBase)) & (( \
       (uint16_t)(~(uint16_t)I2C_SMB_ALERTEN_MASK)) & (( \
       (uint16_t)(~(uint16_t)I2C_SMB_SHTF2_MASK)) & ( \
       (uint16_t)(~(uint16_t)I2C_SMB_SLTF_MASK)))))) | ( \
      (uint16_t)((uint16_t)(State) << I2C_SMB_ALERTEN_SHIFT))) \
  )

/* ----------------------------------------------------------------------------
   -- EnableSecondI2CAddress
   ---------------------------------------------------------------------------- */

/**
 * Enables or disables SMBus device default address.
 * @param peripheralBase Peripheral base address.
 * @param State Parameter specifying if SMBus device default address will be
 *        enabled or disabled.
 */
#define I2C_PDD_EnableSecondI2CAddress(peripheralBase, State) ( \
    I2C_SMB_REG(peripheralBase) = \
     (uint16_t)(( \
      (uint16_t)(( \
       I2C_SMB_REG(peripheralBase)) & (( \
       (uint16_t)(~(uint16_t)I2C_SMB_SIICAEN_MASK)) & (( \
       (uint16_t)(~(uint16_t)I2C_SMB_SHTF2_MASK)) & ( \
       (uint16_t)(~(uint16_t)I2C_SMB_SLTF_MASK)))))) | ( \
      (uint16_t)((uint16_t)(State) << I2C_SMB_SIICAEN_SHIFT))) \
  )

/* ----------------------------------------------------------------------------
   -- SetSCLTimeoutBusClockSource
   ---------------------------------------------------------------------------- */

/**
 * Sets clock source of the timeout counter to Bus clock or Bus clock/64
 * frequency.
 * @param peripheralBase Peripheral base address.
 * @param ClockSource SCL timeout BUS clock source value. The user should use
 *        one from the enumerated values.
 */
#define I2C_PDD_SetSCLTimeoutBusClockSource(peripheralBase, ClockSource) ( \
    I2C_SMB_REG(peripheralBase) = \
     (uint16_t)(( \
      (uint16_t)(( \
       I2C_SMB_REG(peripheralBase)) & (( \
       (uint16_t)(~(uint16_t)I2C_SMB_TCKSEL_MASK)) & (( \
       (uint16_t)(~(uint16_t)I2C_SMB_SHTF2_MASK)) & ( \
       (uint16_t)(~(uint16_t)I2C_SMB_SLTF_MASK)))))) | ( \
      (uint16_t)(ClockSource))) \
  )

/* ----------------------------------------------------------------------------
   -- GetSCLTimeoutInterruptFlags
   ---------------------------------------------------------------------------- */

/**
 * Returns a value that represents a mask of active (pending) interrupts.
 * @param peripheralBase Peripheral base address.
 * @return Use constants from group "SCL timeout flags constants (for
 *         GetSCLTimeoutInterruptFlags, ClearSCLTimeoutInterruptFlags macros)." for
 *         processing return value.
 */
#define I2C_PDD_GetSCLTimeoutInterruptFlags(peripheralBase) ( \
    (uint16_t)(( \
     I2C_SMB_REG(peripheralBase)) & ( \
     (uint16_t)(I2C_SMB_SLTF_MASK | (I2C_SMB_SHTF1_MASK | I2C_SMB_SHTF2_MASK)))) \
  )

/* ----------------------------------------------------------------------------
   -- ClearSCLTimeoutInterruptFlags
   ---------------------------------------------------------------------------- */

/**
 * Clears interrupt flags of interrupts specified by Mask.
 * @param peripheralBase Peripheral base address.
 * @param Mask Mask of interrupt clear requests. Use constants from group "SCL
 *        timeout flags constants (for GetSCLTimeoutInterruptFlags,
 *        ClearSCLTimeoutInterruptFlags macros).".
 */
#define I2C_PDD_ClearSCLTimeoutInterruptFlags(peripheralBase, Mask) ( \
    I2C_SMB_REG(peripheralBase) = \
     (uint16_t)(( \
      (uint16_t)(( \
       I2C_SMB_REG(peripheralBase)) & ( \
       (uint16_t)(~(uint16_t)(I2C_SMB_SLTF_MASK | I2C_SMB_SHTF2_MASK))))) | ( \
      (uint16_t)(Mask))) \
  )

/* ----------------------------------------------------------------------------
   -- EnableSCLTimeoutInterrupt
   ---------------------------------------------------------------------------- */

/**
 * Enables SCL high and SDA low timeout interrupt.
 * @param peripheralBase Peripheral base address.
 */
#define I2C_PDD_EnableSCLTimeoutInterrupt(peripheralBase) ( \
    I2C_SMB_REG(peripheralBase) = \
     (uint16_t)(( \
      (uint16_t)(I2C_SMB_REG(peripheralBase) | I2C_SMB_SHTF2IE_MASK)) & (( \
      (uint16_t)(~(uint16_t)I2C_SMB_SHTF2_MASK)) & ( \
      (uint16_t)(~(uint16_t)I2C_SMB_SLTF_MASK)))) \
  )

/* ----------------------------------------------------------------------------
   -- DisableSCLTimeoutInterrupt
   ---------------------------------------------------------------------------- */

/**
 * Disables SCL high and SDA low timeout interrupt.
 * @param peripheralBase Peripheral base address.
 */
#define I2C_PDD_DisableSCLTimeoutInterrupt(peripheralBase) ( \
    I2C_SMB_REG(peripheralBase) &= \
     (uint16_t)(( \
      (uint16_t)(~(uint16_t)I2C_SMB_SHTF2IE_MASK)) & (( \
      (uint16_t)(~(uint16_t)I2C_SMB_SHTF2_MASK)) & ( \
      (uint16_t)(~(uint16_t)I2C_SMB_SLTF_MASK)))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadSMBusControlAndStatusReg
   ---------------------------------------------------------------------------- */

/**
 * Reads SMBus control and status register.
 * @param peripheralBase Peripheral base address.
 */
#define I2C_PDD_ReadSMBusControlAndStatusReg(peripheralBase) ( \
    I2C_SMB_REG(peripheralBase) \
  )

/* ----------------------------------------------------------------------------
   -- WriteSMBusControlAndStatusReg
   ---------------------------------------------------------------------------- */

/**
 * Writes new value specified by the Value parameter into SMBus control and
 * status register.
 * @param peripheralBase Peripheral base address.
 * @param Value Value to be written to the SMBus control and status register.
 */
#define I2C_PDD_WriteSMBusControlAndStatusReg(peripheralBase, Value) ( \
    I2C_SMB_REG(peripheralBase) = \
     (uint16_t)(Value) \
  )

/* ----------------------------------------------------------------------------
   -- SetSMBusSlaveAddress
   ---------------------------------------------------------------------------- */

/**
 * Sets SMBus address to the address register.
 * @param peripheralBase Peripheral base address.
 * @param SMBusSlaveAddrValue SMBus slave address value[0..127].
 */
#define I2C_PDD_SetSMBusSlaveAddress(peripheralBase, SMBusSlaveAddrValue) ( \
    I2C_A2_REG(peripheralBase) = \
     (uint16_t)(( \
      (uint16_t)(I2C_A2_REG(peripheralBase) & (uint16_t)(~(uint16_t)I2C_A2_SAD_MASK))) | ( \
      (uint16_t)((uint16_t)(SMBusSlaveAddrValue) << I2C_A2_SAD_SHIFT))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadAddress2Reg
   ---------------------------------------------------------------------------- */

/**
 * Reads address 2 register.
 * @param peripheralBase Peripheral base address.
 */
#define I2C_PDD_ReadAddress2Reg(peripheralBase) ( \
    I2C_A2_REG(peripheralBase) \
  )

/* ----------------------------------------------------------------------------
   -- WriteAddress2Reg
   ---------------------------------------------------------------------------- */

/**
 * Writes new value specified by the Value parameter into address 2 register.
 * @param peripheralBase Peripheral base address.
 * @param Value Value to be written to the address 2 register.
 */
#define I2C_PDD_WriteAddress2Reg(peripheralBase, Value) ( \
    I2C_A2_REG(peripheralBase) = \
     (uint16_t)(Value) \
  )

/* ----------------------------------------------------------------------------
   -- SetSCLLowTimeout
   ---------------------------------------------------------------------------- */

/**
 * Sets SCL low timeout value that determines the timeout period of SCL low.
 * @param peripheralBase Peripheral base address.
 * @param SCLLowTimeoutValue SCL low timeout value[0..65535].
 */
#define I2C_PDD_SetSCLLowTimeout(peripheralBase, SCLLowTimeoutValue) ( \
    (I2C_SLTL_REG(peripheralBase) = \
     (uint16_t)(( \
      (uint16_t)(I2C_SLTL_REG(peripheralBase) & (uint16_t)(~(uint16_t)I2C_SLTL_SSLT_MASK))) | ( \
      (uint16_t)((uint16_t)(SCLLowTimeoutValue) & 0xFFU)))), \
    (I2C_SLTH_REG(peripheralBase) = \
     (uint16_t)(( \
      (uint16_t)(I2C_SLTH_REG(peripheralBase) & (uint16_t)(~(uint16_t)I2C_SLTH_SSLT_MASK))) | ( \
      (uint16_t)((uint16_t)(SCLLowTimeoutValue) >> 8U)))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadSclLowTimeoutHighReg
   ---------------------------------------------------------------------------- */

/**
 * Reads SCL low timeout high register.
 * @param peripheralBase Peripheral base address.
 */
#define I2C_PDD_ReadSclLowTimeoutHighReg(peripheralBase) ( \
    I2C_SLTH_REG(peripheralBase) \
  )

/* ----------------------------------------------------------------------------
   -- WriteSclLowTimeoutHighReg
   ---------------------------------------------------------------------------- */

/**
 * Writes new value specified by the Value parameter into SCL low timeout high
 * register.
 * @param peripheralBase Peripheral base address.
 * @param Value Value to be written to the SCL low timeout high register.
 */
#define I2C_PDD_WriteSclLowTimeoutHighReg(peripheralBase, Value) ( \
    I2C_SLTH_REG(peripheralBase) = \
     (uint16_t)(Value) \
  )

/* ----------------------------------------------------------------------------
   -- ReadSclLowTimeoutLowReg
   ---------------------------------------------------------------------------- */

/**
 * Reads SCL low timeout high register.
 * @param peripheralBase Peripheral base address.
 */
#define I2C_PDD_ReadSclLowTimeoutLowReg(peripheralBase) ( \
    I2C_SLTL_REG(peripheralBase) \
  )

/* ----------------------------------------------------------------------------
   -- WriteSclLowTimeoutLowReg
   ---------------------------------------------------------------------------- */

/**
 * Writes new value specified by the Value parameter into SCL low timeout low
 * register.
 * @param peripheralBase Peripheral base address.
 * @param Value Value to be written to the SCL low timeout low register.
 */
#define I2C_PDD_WriteSclLowTimeoutLowReg(peripheralBase, Value) ( \
    I2C_SLTL_REG(peripheralBase) = \
     (uint16_t)(Value) \
  )

#endif // _I2C_H_
