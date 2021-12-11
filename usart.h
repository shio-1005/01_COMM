/* 
 * File:   usart.h
 * Author: shiohitomi
 *
 * Created on November 4, 2021, 3:29 AM
 */

#ifndef USART_H
#define	USART_H

/*---Define---*/
#define _USART_TX_CONFIG             	    0b00100100                          //TX9=1(8bit送信),TXEN=1(送信許可),SYNC=0(非同期モード),BRGH=1(高速モード),
#define _USART_RX_CONFIG             	    0b10010000                          //SPEN=1(シリアルポート許可),CREN=1(連続受信許可),
#define _USART_BAUDRATE_H_9600         	    0x02                                //9600bpsに設定
#define _USART_BAUDRATE_L_9600         	    0x08                                //9600bpsに設定                         
#define _XTAL_FREQ                   		20000000
#define _RX_STRAGE_SIZE              		16


/*--Prototype--*/
void _usart_init(void);
void _putch(unsigned char c);
void usart_puts(char* s);
void usart_save(void);
char* usart_get(void);

#endif
