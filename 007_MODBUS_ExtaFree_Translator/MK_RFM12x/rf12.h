/*
 *	rf12.h
 *
 *  Developer:
 *     Benedikt K.
 *     Juergen Eckert
 *
 *  Version: 2.0.1
 *
 *  MODYFIKACJE: 2011-10-06
 *       Autor: Miros�aw Karda�
 *
 */

#ifndef __RFM12_H
#define __RFM12_H

/* konfiguracja pin�w soft-SPI */
#define RF_PORT		PORTB
#define RF_DDR		DDRB
#define RF_PIN		PINB
#define SDI		3	// MOSI	- wyj�cie
#define SCK		5	// SCK
#define CS		2	// SS
#define SDO		4	// MISO	- wej�cie

#define RF_IRQDDR	DDRD
#define RF_IRQPIN	PIND
#define IRQ		3		// mo�na u�y� INT0, INT1 lub INT2
					// INT2 - po lekkich zmianach kodu


/*****************************************************************/
/*!
 * RF12_UseIRQ = 1 - u�ywamy przerwa�, funkcje nieblokukj�ce
 * RF12_UseIRQ = 0 - przerwania wy��czone, funkcje Blokukj�ce
 */
#define RF12_UseIRQ 1
/*****************************************************************/

/*! Oczekiwanie na zako�czenie nadawania z timeoutem
 *
 * 		WAIT_FOR_SEND = 0 - wy��czone
 * 		WAIT_FOR_SEND = 1 - w��czone
 *
 */
#define WAIT_FOR_SEND 0

#if RF12_UseIRQ == 1
/* RF12 wielko�� bufora - istotne je�li korzystamy z przerwa� */
#define RF12_DataLength	20		//max length 243
#endif


/* ------ */

enum ZKR {ZAKRES_433MHZ=16, ZAKRES_868MHZ=32, ZAKRES_915MHZ=48};


/*!
 * Zakres(MHz)	Rozdzielczos�(KHz)	Min(MHz)	Max(MHz)
 *   433			2.5				 430.24		 439,75
 * 	 868			5.0				 860,48		 879,51
 * 	 915			7.5				 900,72		 929,37
 */





#define RxBW400		1
#define RxBW340		2
#define RxBW270		3
#define RxBW200		4
#define RxBW134		5
#define RxBW67		6

#define TxBW15		0
#define TxBW30		1
#define TxBW45		2
#define TxBW60		3
#define TxBW75		4
#define TxBW90		5
#define TxBW105		6
#define TxBW120		7

#define LNA_0		0
#define LNA_6		1
#define LNA_14		2
#define LNA_20		3

#define RSSI_103	0
#define RSSI_97		1
#define RSSI_91		2
#define RSSI_85		3
#define RSSI_79		4
#define RSSI_73		5
#define RSSI_67		6
#define	RSSI_61		7

#define PWRdB_0		0
#define PWRdB_3		1
#define PWRdB_6		2
#define PWRdB_9		3
#define PWRdB_12	4
#define PWRdB_15	5
#define PWRdB_18	6
#define PWRdB_21	7




// makra do obliczania cz�stotliwo�ci (kana�u)
#define RF12FREQ433(freq)	((freq-430.24)/0.0025)
#define RF12FREQ868(freq)	((freq-860.48)/0.0050)
#define RF12FREQ915(freq)	((freq-900.72)/0.0075)


typedef union {
	uint8_t stat;
	struct {
		uint8_t Rx:1;
		uint8_t Tx:1;
		uint8_t New:1;
	};
} RF12_STAT;


extern volatile RF12_STAT RF12_status;


//! transmisja s�owa (word) do/z modu�u rfm12
uint16_t rf12_trans(uint16_t wert);

//! inicjalizacja rfm12
void rf12_init( void );

//! ustalamy zakres cz�stotliwo�ci: 433MHz, 868MHz lub 915MHz
void rf12_setrange( enum ZKR zakres );

//! wy��czamy WakeUp Timer wbudowany w modu� RFM12
void rf12_disablewakeuptimer( void );

//! ustawienie cz�stotliwo�ci �rodkowej
void rf12_setfreq(uint16_t freq);

//! ustawienie pr�dko�ci transmisji w bodach (set baudrate)
void rf12_setbaud(uint16_t baud);

//! ustawienie mocy nadajnika i FSK
void rf12_setpower(uint8_t power, uint8_t mod);

//! ustawienia odbiornika
void rf12_setbandwidth(uint8_t bandwidth, uint8_t gain, uint8_t drssi);


#if RF12_UseIRQ == 0
//! wys�anie ramki danych z buforu *data o wielko�ci size
void rf12_txdata( char *data, uint8_t size );

//! odbi�r ramki danych do bufora, rezultat - ilo�� odebranych bajt�w
uint8_t rf12_rxdata( char *data);

//! czekaj na gotowo�� FIFO (odbi�r/nadawanie)
void rf12_ready(void);
#endif

#if RF12_UseIRQ == 1

//! start odbioru ramki danych
uint8_t rf12_rxstart(void);

//! odczytanie do bufora gdy zostanie poprawnie odebrana
uint8_t rf12_rxfinish( uint8_t *data);

//! start transmisji/nadawania ramki z bufora o wielko�ci size
uint8_t rf12_txstart( uint8_t *data, uint8_t size );


//! zatrzymanie wszystkich operacji nadwania i odbioru
void rf12_allstop(void);
#endif


#endif
