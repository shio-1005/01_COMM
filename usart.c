/*************************************************************************
 * File         :usart.c                       				 
 * Date         :2020/1/15						 
 * Summary      :Use for usart(Serial communication)			
 * CPU type     :PIC16F886						 
 * Compiler     :XC8							 
 * Author       :Akaru Oishi						 
 * Description  :使用時はINTCONbitsのGIE,PEIEをセットすること		 
 *              :TRISCを出力に指定すること				 
 *              :割り込み処理ルーチンに下記を張り付けること		 
 *              :送信側の送信デリミタにLFを含めること			 
 * ***********************************************************************
 * ---割り込み受信用コピペ---						 
 * if(PIR1bits.RCIF){							
 *       usart_save();							 
 * }									 
 ************************************************************************/


/*---File Include---*/
#include <xc.h>
#include <stdio.h>
#include <string.h>
#include "usart.h"


/*---Variable---*/
char strage[_RX_STRAGE_SIZE];


/*---Initiation---*/
void _usart_init(void)
{
    TXSTA = _USART_TX_CONFIG;                                                   //シリアル通信の送信設定
    RCSTA = _USART_RX_CONFIG;                                                   //シリアル通信の受信設定
    SPBRGH = _USART_BAUDRATE_H_9600;                                            //ボーレートの設定
    SPBRG = _USART_BAUDRATE_L_9600;                                             //ボーレートの設定
    BAUDCTLbits.BRG16 = 1;                                                      //SPBRGレジスタの16bit指定をイネーブル
    INTCONbits.GIE = 1;                                                         //割り込み設定
    INTCONbits.PEIE = 1;                                                        //割り込み設定
    PIE1bits.RCIE  = 1;                                                         //受信割り込みをイネーブル
    PIR1bits.RCIF = 0;                                                          //受信割り込みフラグをリセット、この行たぶんいらん
}

/*---Transmission---*/
void _putch(unsigned char c)
{
    while(!TXIF);
    TXREG = c;
}

void usart_puts(char* s)
{
    while(*s){
        putch(*s);
        s++;
        __delay_ms(10);
    }

}


/*---Save---*/
void usart_save(void){
    if(!strchr(strage,'\n')){                                                   //strageにLFがなければ引き続きstrageに追加
        char* p1 = strage;
        char* p2 = strage;
        
        while(*p1++ != '\0'){
            p2++;
        }
        *p2 = RCREG;
    }
    else{                                                                       //無ければstrageを初期化&新メッセージ
        memset(strage,'\0',_RX_STRAGE_SIZE);
        *strage = RCREG;
    }
}

/*---Get---*/
char* usart_get(void){
    if(!strchr(strage,'\n') || strage[0] == '\r' || strage[0] == '\n'){        //打ち込み途中（LF無し）もしくは空のメッセージの場合getしない
        return 0;
    }
    else{
        return strage;
    }
}
