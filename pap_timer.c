//Control de un PAP mediante Timer
#define	F_CPU 16000000

#define brate0 9600

#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>

unsigned int T,P,PA;

volatile int ACTIVADO;

#define TMIN 10
#define TMAX 60000
#define PMIN 0
#define PMAX 30000
#define TOP 3749

uint8_t indcom;				// índice para llenar el buffer de recepción
char comando[30];			// buffer de recepción

void InterpretaComando(void);	// Función para interpretar comando que llega por puerto serie
void mi_UART_Init( unsigned int);
int mi_putc(char, FILE *stream);		// Función para transmitir caracter por UART
int mi_getc(FILE *stream);				// Función para recibir caracter por UART
void avanza(void);
void retrocede(void);

// Las 2 definiciones siguientes son para usar getc y putc para E/S de caracteres por UART
#define getc() mi_getc(&uart_io)		// redefine la primitiva de entrada como  función recibir por UART
#define putc(x) mi_putc(x,&uart_io)		// redefine la primitiva de salida como función transmitir por UART

FILE uart_io = FDEV_SETUP_STREAM(mi_putc, mi_getc, _FDEV_SETUP_RW); // Declara un tipo stream de E/S

//---------------------------------------------------------------------------------------------
//		Inicializa UART
void mi_UART_Init( unsigned int ubrr)
{
	UBRR0 = F_CPU/16/ubrr-1;				// Configura baudrate. Ver en sección UART de datasheet
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);			// Habilita bits TXEN0 y RXEN0
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);		// USBS0=1 2 bits stop, UCSZxx=3 8 bits
}

//---------------------------------------------------------------------------------------------
//		Pone caracter en UART. *stream es un parámetro usado solo para igualar los parámetros en stdio
int mi_putc(char c, FILE *stream)
{
	while(!(UCSR0A & (1<<UDRE0)) ); // Espera mientras el bit UDRE0=0 (buffer de transmisión ocupado)
	UDR0 = c;						// Cuando se desocupa, UDR0 puede recibir el nuevo dato c a trasmitir
	return 0;
}

//---------------------------------------------------------------------------------------------
//	Recibe caracter de la UART. *stream es un parámetro usado solo para igualar los parámetros en stdio
int mi_getc(FILE *stream)
{
	while ( !(UCSR0A & (1<<RXC0)) );// Espera mientras el bit RXC0=0 (recepción incompleta)
	return UDR0;					// Cuando se completa, se lee UDR0
}

//---------------------------------------------------------------------------------------------
//		Analiza los elementos del buffer de recepción

void InterpretaComando(void)
{
	int aux;
	switch(comando[0])		// Analiza primer Byte del buffer.
	{
		case 'P':                            //posicion absoluta en pasos
			if (comando[1]){
				aux = atoi(&comando[1]);
				if (aux<=PMAX && aux>=PMIN ) {
					P=aux;
				}
			}
			printf("P%d\r\n",P);
			break;
		case 'T':		                      //tiempo entre pasos
			if(comando[1]){
				aux = atoi(&comando[1]);
				if (aux>=TMIN && aux<=TMAX){
					//ICR1=(aux*TOP)/TMAX;
					uint32_t tmp1;
					tmp1= ((uint32_t)aux*(uint32_t)TOP)/TMAX;
					ICR1= (uint16_t)tmp1;
					OCR1A=ICR1/2;
				}
			}
			printf("T:%u\r\n",ICR1);
			printf("T/2:%u\r\n",OCR1A);
			break;
		default:
			break;
	}
}

void avanza(void){
	PORTB |=(1<<PORTB4);
	TCCR1A|=(1<<COM1A1);  //activa salida OC1A
	PA++;
}
void retrocede(void){
	PORTB &= ~(1<<PORTB4);
	TCCR1A|=(1<<COM1A1);   //activa salida OC1A
	PA--;
}

ISR(USART_RX_vect)
{
	char dato;
	dato=getc();
	switch(dato)
	{
		case ':':				// Delimitador de inicio
			indcom=0;				// Inicializa índice de buffer de recepción
			break;
		case '\r':				// Delimitador de final
			comando[indcom]=0;	// coloca \0 luego del último caracter recibido antes de \r
			InterpretaComando();// Llama a función intérprete de comandos
			break;
		default:				// Todo lo que está entre delimitadores, Ej. 'T','3','4','2'
			comando[indcom++]=dato; // Guarda en elemento del buffer e incrementa indcom para apuntar a siguiente
			break;
	}
}

ISR(TIMER1_OVF_vect){  //funcion de interrupcion del timer
    ACTIVADO=1;
}

int main(void)
{
	mi_UART_Init(brate0);
	stdout = stdin = &uart_io;  // El stream (FILE) uart_io es la E/S estándar, es decir para putc y getc

	DDRB = (1<<DDB1)|(1<<DDB4);	     // Salida en OC1A(9),Led (PORTB4)

	//****Configuracion del Timer*****
	TCCR1A=(1<<COM1A1)|(1<<WGM11);                                     //OC1A se inicializa apagado, y configuracion del
	TCCR1B=(1<<WGM12)|(1<<WGM13)|(1<<CS12);	               //modo en 14, con prescaler 256
	TIMSK1=(1<<TOIE1);                                     //Habilitacion de interrupciones por overflow

	//****Inicializacion de varibles****
	T=30000;
	PA=0;
	P=0;
	ICR1=3749;         //65536-1 Este es el top al principio
	OCR1A=ICR1/2;
	ACTIVADO=1;

	printf("PWM PAP Interrupcion\r\n");
	indcom=0;
	UCSR0B|= (1<<RXCIE0);	// Interrupcion Rx UART0
	sei();					// Interrupcion global

	while(1){
		if (ACTIVADO==1){
			if (P>PA){
				avanza();
			}
			else if (P<PA){
				retrocede();
			}
			else{
	            TCCR1A&=~(1<<COM1A1);  //Apaga la salida OC1A
				TCCR1A&=~(1<<COM1A0);
	        }
			ACTIVADO=0;
		}
    }
}


//Control de un PAP mediante Timer
/*//Control de un PAP mediante Timer
#define	F_CPU 16000000

#define brate0 9600

#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>
#include "UART_ATmega0.h"


unsigned int T,P,PA;

volatile int ACTIVADO;

#define TMIN 10
#define TMAX 60000
#define PMIN 0
#define PMAX 30000
#define TOP 3749

uint8_t indcom;				// índice para llenar el buffer de recepción
char comando[30];			// buffer de recepción

void InterpretaComando(void);
void avanza(void);
void retrocede(void);

FILE uart_io = FDEV_SETUP_STREAM(mi_putc, mi_getc, _FDEV_SETUP_RW); // Declara un tipo stream de E/S

void InterpretaComando(void)
{
	int aux;
	switch(comando[0])		// Analiza primer Byte del buffer.
	{
		case 'P':                            //posicion absoluta en pasos
			if (comando[1]){
				aux = atoi(&comando[1]);
				if (aux<=PMAX && aux>=PMIN ) {
					P=aux;
				}
			}
			printf("P%d\r\n",P);
			break;
		case 'T':		                      //tiempo entre pasos
			if(comando[1]){
				aux = atoi(&comando[1]);
				if (aux>=TMIN && aux<=TMAX){
					//ICR1=(aux*TOP)/TMAX;
					uint32_t tmp1;
					tmp1= ((uint32_t)aux*(uint32_t)TOP)/TMAX;
					ICR1= (uint16_t)tmp1;
					OCR1A=ICR1/2;
				}
			}
			printf("T:%u\r\n",ICR1);
			printf("T/2:%u\r\n",OCR1A);
			break;
		default:
			break;
	}
}

void avanza(void){
	PORTB |=(1<<PORTB4);
	TCCR1A|=(1<<COM1A1);  //activa salida OC1A
	PA++;
}
void retrocede(void){
	PORTB &= ~(1<<PORTB4);
	TCCR1A|=(1<<COM1A1);   //activa salida OC1A
	PA--;
}

ISR(USART_RX_vect)
{
	char dato;
	dato=getc();
	switch(dato)
	{
		case ':':				// Delimitador de inicio
			indcom=0;				// Inicializa índice de buffer de recepción
			break;
		case '\r':				// Delimitador de final
			comando[indcom]=0;	// coloca \0 luego del último caracter recibido antes de \r
			InterpretaComando();// Llama a función intérprete de comandos
			break;
		default:				// Todo lo que está entre delimitadores, Ej. 'T','3','4','2'
			comando[indcom++]=dato; // Guarda en elemento del buffer e incrementa indcom para apuntar a siguiente
			break;
	}
}

ISR(TIMER1_OVF_vect){  //funcion de interrupcion del timer
    ACTIVADO=1;
}

int main(void)
{
	mi_UART_Init0(9600,0,1);
	stdout = stdin = &uart_io;  // El stream (FILE) uart_io es la E/S estándar, es decir para putc y getc

	DDRB = (1<<DDB1)|(1<<DDB4);	     // Salida en OC1A(9),Led (PORTB4)

	//****Configuracion del Timer*****
	TCCR1A=(1<<COM1A1)|(1<<WGM11);                                     //OC1A se inicializa apagado, y configuracion del
	TCCR1B=(1<<WGM12)|(1<<WGM13)|(1<<CS12);	               //modo en 14, con prescaler 256
	TIMSK1=(1<<TOIE1);                                     //Habilitacion de interrupciones por overflow

	//****Inicializacion de varibles****
	T=30000;
	PA=0;
	P=0;
	ICR1=3749;         //(0.06 s)*(16000000/256)=3750----->N-1=3749
	OCR1A=ICR1/2;
	ACTIVADO=1;

	printf("PWM PAP Interrupcion\r\n");
	indcom=0;
	UCSR0B|= (1<<RXCIE0);	// Interrupcion Rx UART0
	sei();					// Interrupcion global

	while(1){
		if (ACTIVADO==1){
			if (P>PA){
				avanza();
			}
			else if (P<PA){
				retrocede();
			}
			else{
	            TCCR1A&=~(1<<COM1A1);  //Apaga la salida OC1A
				TCCR1A&=~(1<<COM1A0);
	        }
			ACTIVADO=0;
		}
    }
}
*/
