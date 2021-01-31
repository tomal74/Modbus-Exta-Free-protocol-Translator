/*
 *	rf12.c
 *
 *  Developer:
 *     Benedikt K.
 *     Juergen Eckert
 *
 *  Version: 2.0.1
 *
 *  MODYFIKACJE: 2011-10-06
 *       Autor: Miros³aw Kardaœ
 *
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>

#include "rf12.h"

/*! ***************************************************************
 * 	FUNKCJE WSPÓLNE
 *  ***************************************************************/


/*! Inicjalizacja modu³u transcievera RFM12 */
void rf12_init( void ) {
	// inicjalizacja pinów soft-SPI
	RF_DDR 	|= (1<<SDI)|(1<<SCK)|(1<<CS);
	RF_DDR 	&= ~(1<<SDO);
	RF_PORT |= (1<<CS);
#if RF12_UseIRQ == 1
	RF_PORT |= (1<<SDO);
#endif

	_delay_ms(100);		// oczekiwanie na reset modu³u

#if RF12_UseIRQ == 1
	// jeœli obs³uga na przerwaniach
	RF12_status.Rx 	= 0;
	RF12_status.Tx 	= 0;
	RF12_status.New = 0;

	// inicjalizacja przerwania asynchronicznego INT2
	RF_IRQDDR 	&= ~(1<<IRQ);
	RF_PORT 	|= (1<<IRQ);
	GICR 		|= (1<<INT2);
	MCUCSR 		&= ~(1<<ISC2);	// zbocze opadaj¹ce

	// inicjalizacja przerwania synchronicznego INT0 / INT1
//	RF_IRQDDR 	&= ~(1<<IRQ);
//	RF_PORT 	|= (1<<IRQ);
//	GICR 		|= (1<<INT0); //(1<<INT1);
//	MCUCR 		|= (1<<ISC01); //(1<<ISC11);	// zbocze opadaj¹ce

#endif

}


/*! Transmisja s³owa do i z modu³u RFM12 */
uint16_t rf12_trans(uint16_t val)
{
	uint16_t ret_val = 0;
	uint8_t i;

	RF_PORT &= ~(1<<CS);
	for (i=0; i<16; i++)
	{
		if (val & 0x8000) RF_PORT |= (1<<SDI);
		else RF_PORT &= ~(1<<SDI);

		ret_val <<= 1;
		if (RF_PIN&(1<<SDO)) ret_val |= 1;
		RF_PORT |= (1<<SCK);
		val <<= 1;
		asm("nop");
		asm("nop");
		RF_PORT &= ~(1<<SCK);
	}
	RF_PORT |= (1<<CS);

	return ret_val;
}

/*! Obliczanie sumy kontrolnej CRC16 */
uint16_t crcUpdate(uint16_t crc, uint8_t serialData) {
	uint16_t tmp;
	uint8_t j;

	tmp = serialData << 8;
        for (j=0; j<8; j++)	{
            if((crc^tmp) & 0x8000) crc = (crc<<1) ^ 0x1021;
            else crc = crc << 1;
            tmp = tmp << 1;
        }
	return crc;
}

void rf12_setrange( enum ZKR zakres ) {
	rf12_trans( 0x80C7 | zakres );	// Enable FIFO , set range
}

void rf12_setbandwidth(uint8_t bandwidth, uint8_t gain, uint8_t drssi) {
	rf12_trans(0x9000|((bandwidth&7)<<5)|((gain&3)<<3)|(drssi&7));
}

void rf12_setfreq(uint16_t freq) {
	if (freq<96)				// 430,2400MHz minimum
		freq=96;
	else if (freq>3903)			// 439,7575MHz maximum
		freq=3903;
	rf12_trans(0xA000|freq);
}

void rf12_setbaud(uint16_t baud) {
	if (baud<663)
		return;
	if (baud<5400)					// Baudrate= 344827,58621/(R+1)/(1+CS*7)
		rf12_trans(0xC680|((43104/baud)-1));
	else
		rf12_trans(0xC600|((344828UL/baud)-1));
}

void rf12_disablewakeuptimer( void ) {
	rf12_trans(0xE000);				// wy³¹czamy wakeuptimer
}


void rf12_setpower(uint8_t power, uint8_t mod) {
	rf12_trans(0x9800|(power&7)|((mod&15)<<4));
}

/*! ***************************************************************
 * 	FUNKCJE WSPÓLNE - KONIEC
 *  ***************************************************************/



/*! ***************************************************************
 * 	FUNKCJE NIEBLOKUJ¥CE - OBS£UGA RFM12 NA PRZERWANIACH
 *  ***************************************************************/
#if RF12_UseIRQ == 1

/*! Zmienne globalne */
volatile RF12_STAT RF12_status;				// struktura statusów rfm12
volatile uint8_t RF12_Index = 0;
volatile uint8_t RF12_Data[ RF12_DataLength ];	// +10 nadmiarowo na pozosta³e czêœci ramki


/*! **************** OBS£UGA PRZERWANIA INTx ******************** */
ISR(INT2_vect) {

	if(RF12_status.Rx) {
		if(RF12_Index < RF12_DataLength) {
			RF12_Data[RF12_Index++] = rf12_trans(0xB000) & 0x00FF;
		} else {
			rf12_trans(0x8208);
			RF12_status.Rx = 0;
			RF12_status.New = 1;
			return; 				// na pewno bêdzie b³êdna ramka
		}
		if(RF12_Index >= 18) {

			rf12_trans(0x8208);
			RF12_status.Rx = 0;
			RF12_status.New = 1;	// poprawnie zakoñczona ramka
		}
	}
	else
	if(RF12_status.Tx) {
		rf12_trans(0xB800 | RF12_Data[RF12_Index]);
		if(!RF12_Index)	{
			RF12_status.Tx = 0;
			rf12_trans(0x8208);		// TX off
		} else {
			RF12_Index--;
		}
	}
}
/* OBS£UGA PRZERWANIA INTx  - KONIEC */

/*! Funkcja inicjuj¹ca odbiór danych */
uint8_t rf12_rxstart(void) {
	if(RF12_status.New)
		return(1);			//bufor jeszcze nie pusty
	if(RF12_status.Tx)
		return(2);			//trwa nadawanie
	if(RF12_status.Rx)
		return(3);			//trwa odbieranie

	rf12_trans(0x82C8);			// RX on
	// fifo reset
	rf12_trans(0xCA81);			// disable FIFO mode
	rf12_trans(0xCA83);			// enable FIFO mode

	RF12_Index = 0;
	RF12_status.Rx = 1;

	return(0);				//wszystko w porz¹dku
}

/*! Funkcja sprawdzaj¹ca i finalizuj¹ca odbiór kompletnej ramki danych
 *
 *  *data - bufor na odebran¹ ramkê
 *
 *  ret = iloœæ bajtów ramki danych
 *  jeœli B£¥D i:
 *  ret = 255 to oznacza, ¿e jeszcze trwa odbiór ramki
 *  ret = 254 to oznacza, ¿e jeszcze nie odczytaliœmy poprzedniej ramki
 *
 * */
uint8_t rf12_rxfinish( uint8_t *data ) {
	//uint16_t crc, crc_chk = 0;
	uint8_t i, size = 18;
	if(RF12_status.Rx) return(255);		//odbiór jeszcze nie zakoñczony
	if(!RF12_status.New) return(254);	//jeszcze stare dane w buforze

	if( size > RF12_DataLength ) {
		data[0] = 0;
		RF12_status.New = 0;
		return 0; // b³¹d wielkoœci ramki
	}

	//for(i=0; i<size +1 ; i++)
		//crc_chk = crcUpdate(crc_chk, RF12_Data[i]);

	//crc = RF12_Data[i++];
	//crc |= RF12_Data[i] << 8;
	RF12_status.New = 0;

		for(i=0; i<size; i++)
			data[i] = RF12_Data[i];
			RF12_Data[i] = 0;	//zeruj reszte - czyszczenie smieci

		// data[ size ] = 0;		// zakoñczenie ramki zerem
		return( size );			// rozmiar odebranej ramki w bajtach

}

/*! Funkcja inicjalizuj¹ca transmisjê pakietu danych
 *  *data - wskaŸnik na bufor danych
 *  size = 0 gdy transmitujemy C-String (³añcuch tekstowy ASCII)
 *  size > 0 gdy transmitujemy dane binarne
 *
 *  ret = 0 - nadawanie zainicjalizowane poprawnie
 *  B£ÊDY
 *  ret = 1 - trwa nadawanie poprzedniej ramki
 *  ret = 2 - trwa odbieranie ramki
 *  ret = 3 - zbyt du¿a ramka danych
 *
 */
uint8_t rf12_txstart( uint8_t *data, uint8_t size ) {
	uint8_t i, l;

	//if( !size ) size = strlen(data);

	if(RF12_status.Tx)
		return(1);			//nadawanie w toku
	if(RF12_status.Rx)
		return(2);			//odbieranie w toku
	if(size > RF12_DataLength)
		return(3);			//za d³uga ramka danych

	RF12_status.Tx = 1;
	//zwiêkszenie ramki o 10 na potrzeby:
	// preambu³y (3bajty), 2 bajty synchro, 1 bajt rozmiar ramki,
	// sumy CRC16 (2 bajty) oraz na koñcu dwa bajty 0xAA

	RF12_Index = size + 6;			//act -10

	i = RF12_Index;
	RF12_Data[i--] = 0xAA;
	RF12_Data[i--] = 0xAA;
	RF12_Data[i--] = 0xAA;
	RF12_Data[i--] = 0x2D;
	RF12_Data[i--] = 0xD4;

	for(l=0; l<size; l++) {
		RF12_Data[i--] = data[l];
	}

	RF12_Data[i--] = 0xAA;
	RF12_Data[i--] = 0xAA;


	rf12_trans(0x8238);			// TX on

#if WAIT_FOR_SEND == 1
	uint8_t tmout=250;
	while( tmout-- && RF12_status.Tx ) {
		_delay_ms(1);
	}
#endif


	return 0;				//wszystko OK
}

/*! zatrzymanie nadawania i odbioru  */
void rf12_allstop(void) {
	RF12_status.Rx = 0;
	RF12_status.Tx = 0;
	RF12_status.New = 0;
	rf12_trans(0x8208);		//stan bezczynnoœci rfm12
	rf12_trans(0x0000);		//pusty odczyt
}


#endif
/*! ***************************************************************
 * 	FUNKCJE NIEBLOKUJ¥CE - OBS£UGA RFM12 NA PRZERWANIACH - KONIEC
 *  ***************************************************************/







/*! ***************************************************************
 * 	FUNKCJE BLOKUJ¥CE - OBS£UGA RFM12 bez u¿ycia przerwañ
 *  ***************************************************************/
#if RF12_UseIRQ == 0

void rf12_ready(void) {
	RF_PORT &= ~(1<<CS);
	while (!(RF_PIN & (1<<SDO))); // czekaj na gotowoœæ FIFO
}

void rf12_txdata(char *data, uint8_t size) {
	uint8_t i;
	uint16_t crc;

	rf12_trans(0x8238);			// TX on

	if( !size ) size = (strlen(data));

	rf12_ready();
	rf12_trans(0xB8AA);		// bajty preambu³y 2x 0xAA
	rf12_ready();
	rf12_trans(0xB8AA);
	rf12_ready();
	rf12_trans(0xB8AA);
	rf12_ready();
	rf12_trans(0xB82D);		// dwa bajty synchronizacyjne 0x2D i 0xD4
	rf12_ready();
	rf12_trans(0xB8D4);
	rf12_ready();
	rf12_trans(0xB800 | size);
	crc = crcUpdate(0, size);
	for (i=0; i<size; i++) {
		rf12_ready();
		rf12_trans(0xB800 | data[i]);
		crc = crcUpdate(crc, data[i]);
	}
	rf12_ready();
	rf12_trans(0xB800 | (crc & 0x00FF));
	rf12_ready();
	rf12_trans(0xB800 | (crc >> 8));
	rf12_ready();
	rf12_trans(0xB8AA);
	rf12_ready();
	rf12_trans(0xB8AA);
	rf12_ready();

	rf12_trans(0x8208);			// TX off
}


uint8_t rf12_rxdata( char *data ) {
	uint8_t i, number;
	uint16_t crc, crc_chk;

	rf12_trans(0x82C8);			// RX on

	// fifo reset
	rf12_trans(0xCA81);			// disable FIFO mode
	rf12_trans(0xCA83);			// enable FIFO mode

	rf12_ready();
	number = rf12_trans(0xB000) & 0x00FF;
	crc_chk = crcUpdate(0, number);

	for (i=0; i<number; i++) {
		rf12_ready();
		data[i] = ( char) (rf12_trans(0xB000) & 0x00FF);
		crc_chk = crcUpdate(crc_chk, data[i]);
	}

	rf12_ready();
	crc = rf12_trans(0xB000) & 0x00FF;
	rf12_ready();
	crc |=  rf12_trans(0xB000) << 8;

	rf12_trans(0x8208);			// RX off

	if (crc != crc_chk)	{
		number = 0;
		data[0]=0;
	}

	data[number]=0;
	return number;
}
/* ------------------------- */
#endif


