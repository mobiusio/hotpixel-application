#ifndef SPI_H
#define SPI_H
#include "derivative.h"
#include "user_config.h"
#include "stdlib.h"
#include "string.h"
#include "types.h"

void spi1_init (void);
void spi1_deinit (void);
void spi1_set_select (uint_8 s);
uint_8 spi1_byte_xmit (uint_8 tx, uint_8 *rx);

#endif // SPI_H
