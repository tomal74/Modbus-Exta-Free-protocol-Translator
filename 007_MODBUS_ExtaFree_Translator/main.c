/*
 * main_e.c
 *
 *  Created on: 24 list 2020
 *      Author: Tomasz Konieczka
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <avr/pgmspace.h>

#include "MK_RFM12x/rf12.h"
#include "MODBUS/modbus.h"

#define clientAddress 0x01

#define LED_ON ( PORTC &= ~(1<<PC1) )
#define LED_OFF ( PORTC |= (1<<PC1) )

uint8_t RFM12_buf[18];		// bufor odbiorczy
uint8_t RFM12_Tx_buf[18];	// bufor nadawczy

uint8_t motion_det[5];	// tablica czujek ruchu

volatile uint16_t Timer1;
volatile uint8_t first_prog_rotation = 1;

volatile uint8_t instate = 0;
volatile uint8_t outstate = 0;
volatile uint16_t inputRegisters[4];
volatile uint16_t holdingRegisters[12] = { 0, 1, 33 };	// holdingRegisters[0] - wlaczanie parowania nowych urzadzen															// holdingRegisters[1] - nadajnik (pilot 255 kanalowy!!)

uint8_t RFM12_send(void);
void RFM12_user_param_init(void);
uint8_t RFM12_check_frame_sum(uint8_t *RFM12_user_buff);

uint8_t exta_free_sum(uint8_t *RFM12_user_buff);
uint8_t exta_free_check_type(void);
uint8_t exta_free_write_val2holding_reg(volatile uint16_t *user_holding_register);
void exta_free_remote_key_push_frame(uint8_t key_nr, uint8_t key_down_up);

void timer0100us_start(void);
void modbusGet(void);

uint8_t ADC_rand_no(void);


/*
*   Modify the following 3 functions to implement your own pin configurations...
*/
void SetOuts(volatile uint8_t in) {
	PORTD|= (((in & (1<<3))<<4) | ((in & (1<<4))<<1) | ((in & (1<<5))<<1));
	PORTB|= (((in & (1<<0))<<2) | ((in & (1<<1))) | ((in & (1<<2))>>2));
	in=~in;
	PORTB&= ~(((in & (1<<0))<<2) | ((in & (1<<1))) | ((in & (1<<2))>>2));
	PORTD&= ~(((in & (1<<3))<<4) | ((in & (1<<4))<<1) | ((in & (1<<5))<<1));
}

uint8_t ReadIns(void) {
	uint8_t ins=0x00;
	ins|=(PINC&((1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<4)|(1<<5)));
	ins|=(((PIND&(1<<4))<<2)|((PIND&(1<<3))<<4));
	return ins;
}

void io_conf(void) {
	/*
	 Outputs: PB2,PB1,PB0,PD7,PD5,PD6
	 Inputs: PC0, PC1, PC2, PC3, PC4, PC6, PD4, PD3
	*/
	DDRD=0x00;
	DDRB=0x00;
	DDRC=0x00;
	PORTD=0x00;
	PORTB=0x00;
	PORTC=0x00;
	PORTD|=(1<<0);
	DDRD |= (1<<2)|(1<<5)|(1<<6)|(1<<7);
	DDRB |= (1<<0)|(1<<1)|(1<<2)|(1<<3);
}



int main(void)
{
	// Ustawienie pinu PC1 jako wyjście (dioda LED
	DDRC |= (1<<PC1);
	PORTC |= (1<<PC1);

	// Inicjalizacja przerwan od Timer2 tryb CTC - timery programowe
	TCCR2A 	|= (1<<WGM21);			//CTC
	TCCR2A 	|= (1<<CS22)|(1<<CS21)|(1<<CS20);	// preskaler = 1024
	OCR2A 	= 77;					// IRQ 10ms (100Hz)
	TIMSK2 	|= (1<<OCIE2A);			// IRQ CTC ON


	//io_conf();
	sei();
	// ustawienie adresu urządzenia na magistrali - protokół Modbus
	modbusSetAddress(clientAddress);
	// inicjalizacja urządzenia Modbus serial Slave - magistrala RS485
	modbusInit();
	// inicjalizacja modułu RFM12_B z parametrami niezbednymi do komunikacji z systemem exta free
	RFM12_user_param_init();
	// start przerwań od timera0, T_timer0 = 100us, na potrzeby protokołu Modbus
	timer0100us_start();

	// zmienne pomocnicze do zastosowan w transmisji radiowej 868MHz
	uint8_t ret = 0;
	// char tx_stat = 0;

    while(1)
    {
    	// cykliczne czekanie na żądanie ModBus od Mastera
	    modbusGet();

	    // emulowanie pilotow exta free
	    if( holdingRegisters[1] ) {
	    	// odczytujemy nr klawisza
	    	uint8_t key_nr =  holdingRegisters[1];
	    	uint8_t key_up_down = (uint16_t) (holdingRegisters[1] >> 8);
	    	holdingRegisters[1] = 0;
	    	// wypełniamy bufor nadawczy kodem pilota
	    	exta_free_remote_key_push_frame(key_nr, key_up_down);	// 0 - key_down, 1 - key_up
	    	// wyłaczenie nasłuchu radiowego 868MHz
	    	RF12_status.stat = 0x00;
	    	//wysyłamy tą samą ramke dwa razy - zgodnie z protokołem exta free
	    	LED_ON;
	    	// zaczekaj aż ramki zostaną wysłane
	    	while( RFM12_send() ) ;
	    	while( RFM12_send() ) ;
	    	LED_OFF;
	    }

	    if(!Timer1) {
	    	holdingRegisters[3] = 0;
	    }

	    // jezeli nic nie jest odbierane ani wysyłane, zacznij nasłuchiwanie
	    if( !(RF12_status.stat&0x07) ) rf12_rxstart();
	    // jeżli odebrano prawidłową ramke exta free - wpisz dane do rejestrów typu holdingregister (ModBus)
	    if( RF12_status.New ) {
			memset(RFM12_buf, 0, sizeof(RFM12_buf));
			ret = rf12_rxfinish(RFM12_buf);	// sprawdź czy odebrano kompletną ramkę

			if( (ret > 0 && ret < 19) && RFM12_check_frame_sum(RFM12_buf) ) {	// liczba odebranych bajtów i suma kontrolna OK
				exta_free_write_val2holding_reg(holdingRegisters);
			}
	    }

	    first_prog_rotation = 0;
    }
}


uint8_t RFM12_send(void) {
#if RF12_UseIRQ == 1

	return rf12_txstart(RFM12_Tx_buf, 18);

#endif
}

// key_down_up - 0 - nacisniecie, 1 puszczenie
void exta_free_remote_key_push_frame(uint8_t key_nr, uint8_t key_down_up) {
	if(key_down_up > 1) key_down_up = 1;

	static uint8_t step_key;	// kod kroczacy ramki exta free
	if(first_prog_rotation) step_key = ADC_rand_no();
	key_nr += (key_nr-1) + key_down_up;
	// kodowanie na poczatku ramki nr klwisza
	RFM12_Tx_buf[0] = key_nr;
	RFM12_Tx_buf[1] = 0xFF - key_nr;
	RFM12_Tx_buf[2] = key_nr;
	// trzy bajty nr seryjnego nadajnika (pilota)
	RFM12_Tx_buf[3] = 0x14;
	RFM12_Tx_buf[4] = 0x6E;
	RFM12_Tx_buf[5] = 0x5D;
	// kod kroczacy - jeden bajt
	RFM12_Tx_buf[6] = step_key++;
	//stala wartosc dla pilotow exta free - 0x02 0x00 0x00
	RFM12_Tx_buf[7] = 0x02;
	RFM12_Tx_buf[8] = 0x00;
	RFM12_Tx_buf[9] = 0x00;
	// wyliczenie sumy kontrolnej protokołu exta free
	RFM12_Tx_buf[10] = exta_free_sum(RFM12_Tx_buf);
	// zkonczenie ramki exta free zerem
	RFM12_Tx_buf[11] = 0x00;
}


void RFM12_user_param_init(void) {
	//  RFM12 inicjalizacja
	rf12_init();								// inicjalizacja soft SPI i przerwań jeśli są używane
	rf12_setrange( ZAKRES_868MHZ );				// ustawiamy zakres częstotliwości: 433MHz, 868MHz lub 915MHz
	rf12_trans(0xA000|1664); // czestotliwosc f = 868.3199MHz
	rf12_trans(0xC623);  //bitrate 9600
	rf12_setbandwidth(RxBW200, LNA_0, RSSI_61);	// ustawienia odbiornika
	rf12_setpower(PWRdB_0, TxBW90);			// 1mW moc wyjściowa nadajnika, parametry FSK
	rf12_disablewakeuptimer();					// wyłączamy Wakeup Timer
	rf12_trans(0xC4DB); //AFC command
	rf12_trans(0x0000);							// odczyt statusu - start modułu rfm12
	rf12_trans(0xCC77);				// jeśli posiadasz RFM12B (wesja SMD) odblokuj tę linię
}


uint8_t RFM12_check_frame_sum(uint8_t *RFM12_user_buff) {
	uint8_t frame_sum = exta_free_sum(RFM12_user_buff);

	if(frame_sum == RFM12_user_buff[10]) return 1;	// sprawdzamy czy suma wyliczona zgadza sie z odebrana (10 bajt)return
	else return 0;
}

uint8_t exta_free_sum(uint8_t *RFM12_user_buff) {
	uint16_t frame_sum=0;

	for(uint8_t i=0; i<10; i++) { frame_sum += RFM12_user_buff[i]; }	// liczymy sume odebranych bajtow od 0 do 9
	frame_sum = (uint8_t)frame_sum;

	return frame_sum;
}

uint8_t exta_free_check_type(void) {
	if(RFM12_buf[3]==0x42 && RFM12_buf[4]==0x14 && RFM12_buf[5]==0x2C) return 3;  // czujnik ruchu
	else if(RFM12_buf[3]==0x34 && RFM12_buf[4]==0x03 && RFM12_buf[5]==0xE6) return 2;	// czujnik temperatury
	else if(RFM12_buf[3]==0x14 && RFM12_buf[4]==0x6E && RFM12_buf[5]==0x5D) return 1;  // pilot

	return 0;
}

uint8_t exta_free_write_val2holding_reg(volatile uint16_t *user_holding_register) {
	uint8_t exta_type = exta_free_check_type();

	if(exta_type == 2) {
		uint8_t sens_val_sign = RFM12_buf[8] & 0x80;
		uint8_t sens_val, sens_val_dot;

		sens_val = RFM12_buf[8] & 0x7F;
		sens_val_dot = RFM12_buf[9];

		if(RFM12_buf[7] == 0x52 || RFM12_buf[7] == 0x50) sens_val_dot &= ~(1<<0) ; // czujnik, stan baterii OK
		else if(RFM12_buf[7] == 0x5A || RFM12_buf[7] == 0x58) sens_val_dot |= (1<<0);	// czujnik, NISKI stan baterii

		// wartość ujemna (znak na minus)
		if(sens_val_sign) sens_val |= (1<<7);

		user_holding_register[2] = (uint16_t) (sens_val << 8) | sens_val_dot;
	}
	else if(exta_type == 3) { //czujnik ruchu
		Timer1 = 2000;	// ustaw Timer1 na 10s
		user_holding_register[3] = 1;
	}

/*	if(exta_type == 1) {	// pilot
		user_holding_register[3] = RFM12_buf[0];
	}*/

	return 0;
}



void timer0100us_start(void) {
	TCCR0B|=(1<<CS01); //prescaler 8
	TIMSK0|=(1<<TOIE0);
}


void modbusGet(void) {
	if (modbusGetBusState() & (1<<ReceiveCompleted))
	{
		switch(rxbuffer[1]) {
			case fcReadCoilStatus: {
				modbusExchangeBits(&outstate,0,8);
			}
			break;

			case fcReadInputStatus: {
				volatile uint8_t inps = ReadIns();
				modbusExchangeBits(&inps,0,8);
			}
			break;

			case fcReadHoldingRegisters: {
				modbusExchangeRegisters(holdingRegisters,0,12);
			}
			break;

			case fcReadInputRegisters: {
				modbusExchangeRegisters(inputRegisters,0,4);
			}
			break;

			case fcForceSingleCoil: {
				modbusExchangeBits(&outstate,0,8);
				SetOuts(outstate);
			}
			break;

			case fcPresetSingleRegister: {
				modbusExchangeRegisters(holdingRegisters,0,12);
			}
			break;

			case fcForceMultipleCoils: {
				modbusExchangeBits(&outstate,0,8);
				SetOuts(outstate);
			}
			break;

			case fcPresetMultipleRegisters: {
				modbusExchangeRegisters(holdingRegisters,0,12);
			}
			break;

			default: {
				modbusSendException(ecIllegalFunction);
			}
			break;
		}
	}
}

uint8_t ADC_rand_no(void) {

	ADMUX |= (1<<REFS0) | (1<<REFS1);	// ADC - Internal Vref = 2.56
	ADCSRA |= (1<<ADEN) |(1<<ADPS1) | (1<<ADPS2);	// ADC on, ADC prescaler / 64
	ADMUX = (ADMUX & 0xF8) | 0x01;	// Wybór kanału
	ADCSRA |= (1<<ADSC);	// Start pomiaru ADC
	while(ADCSRA & (1<<ADSC));	// Zaczekaj do momenu skończenia pomiaru ADC
	ADCSRA &= ~(1<<ADEN);
	return ADCL;
}




ISR(TIMER2_COMP_vect)
{
	uint16_t n;

	n = Timer1;		/* 100Hz Timer1 */
	if (n) Timer1 = --n;

}


ISR(TIMER0_OVF_vect) { //this ISR is called 9765.625 times per second
	modbusTickTimer();
	TCNT0 = 156;
}

