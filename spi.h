/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef SPI_H
#define	SPI_H

/* Define */
#define _SSPSTAT_CONFIG 0b01000000      //CKE(bit6)が1（立ち上がりで送信）
#define _SSPCON_CONFIG  0b00100000      //SPIマスターモード，クロックはFosc/4，クロックがLowでアイドル
#define _XTAL_FREQ      20000000

/* prototype */
void spi_init(void);
char spi_putc(char c);

#endif
