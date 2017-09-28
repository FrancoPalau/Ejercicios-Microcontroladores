/* Realizar un variador de velocidad PWM a lazo abierto para un motor DC, que acepte comando del tipo :Dnnn, nnn de 0 a 100%.
*/
#define F_CPU 16000000
#define miBrate 9600

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>

// Las 2 definiciones siguientes son para usar getc y putc para E/S de caracteres por UART
#define getc() mi_getc(&uart_io)		// redefine la primitiva de entrada como  funcion recibir por UART
#define putc(x) mi_putc(x,&uart_io)		// redefine la primitiva de salida como funcion transmitir por UART

//Firmas de las funciones
void mi_UART_Init( unsigned int);		// Funcion que inicializa la UART
int mi_putc(char, FILE *stream);		// Funcion para transmitir caracter por UART
int mi_getc(FILE *stream);				// Funcion para recibir caracter por UART
void InterpretaComando(void);
ISR(USART_RX_vect);
ISR(TIMER1_OVF_vect);

uint8_t indcom;
char comando[30];

const uint8_t sinus[256]={
	127,130,133,136,139,143,146,149,152,155,158,161,164,167,170,173,176,179,
	182,184,187,190,193,195,198,200,203,205,208,210,213,215,217,219,221,224,
	226,228,229,231,233,235,236,238,239,241,242,244,245,246,247,248,249,250,
	251,251,252,253,253,254,254,254,254,254,255,254,254,254,254,254,253,253,
	252,251,251,250,249,248,247,246,245,244,242,241,239,238,236,235,233,231,
	229,228,226,224,221,219,217,215,213,210,208,205,203,200,198,195,193,190,
	187,184,182,179,176,173,170,167,164,161,158,155,152,149,146,143,139,136,
	133,130,127,124,121,118,115,111,108,105,102,99,96,93,90,87,84,81,78,75,
	72,70,67,64,61,59,56,54,51,49,46,44,41,39,37,35,33,30,28,26,25,23,21,19,
	18,16,15,13,12,10,9,8,7,6,5,4,3,3,2,1,1,0,0,0,0,0,0,0,0,0,0,0,1,1,2,3,3,
	4,5,6,7,8,9,10,12,13,15,16,18,19,21,23,25,26,28,30,33,35,37,39,41,44,46,
	49,51,54,56,59,61,64,67,70,72,75,78,81,84,87,90,93,96,99,102,105,108,
111,115,118,121,124};

FILE uart_io = FDEV_SETUP_STREAM(mi_putc, mi_getc, _FDEV_SETUP_RW); // Declara un tipo stream de E/S

int main() {
	mi_UART_Init(miBrate);
	stdout = stdin = &uart_io;
	DDRB = (1<<DDB1)|(1<<DDB5);	     // Salida en OC1A(9), Led (PORTB5)
	TCCR1A=(1<<COM1A1)|(1<<COM1B0)|(1<<WGM11);  // Modo 10
	TCCR1B=(1<<WGM13)|(1<<CS11)|(1<<CS10);      //Prescaler en 64
	ICR1=65535;                                 //65536-1 //Este es el top
	//TIMSK1=(1<<TOIE1);                          //Habilitacion de interrupciones por overflow (usado para la sinusoidal)
	printf("PWM para DC (con opcion de sinusoidal)\r\n");
	sei();
	UCSR0B|= (1<<RXCIE0);                       // Habilita interrupciones de UART

	do{
		PORTB^=(1<<PORTB5);                    //Toggle de un led solo para verificar funcionamiento
		_delay_ms(100);
	} while(1);
	return 0;
}

//		Inicializaci0n de UART
void mi_UART_Init( unsigned int brate){
	UBRR0 = F_CPU/16/brate-1;				// Configura baudrate.
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);			// Habilita bits TXEN0 y RXEN0
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);		// USBS0=1 2 bits stop, UCSZxx=3 8 bits
}

//		Pone caracter en UART. *stream es un parametro usado solo para igualar los parametros en stdio
int mi_putc(char c, FILE *stream){
	while(!(UCSR0A & (1<<UDRE0)) ); // Espera mientras el bit UDRE0=0 (buffer de transmision ocupado)
	UDR0 = c;						// Cuando se desocupa, UDR0 puede recibir el nuevo dato c a trasmitir
	return 0;
}

//	Recibe caracter de la UART. *stream es un parametro usado solo para igualar los parametros en stdio
int mi_getc(FILE *stream){
	while ( !(UCSR0A & (1<<RXC0)) );// Espera mientras el bit RXC0=0 (recepci?n incompleta)
	return UDR0;					// Cuando se completa, se lee UDR0
}

ISR(USART_RX_vect){
	char dato;
	dato=getc();
	switch(dato){
		case ':':
		indcom=0;
		break;
		case 13:
		comando[indcom]=0;
		InterpretaComando();
		break;
		default:
		comando[indcom++]=dato;
		break;
	}
}

void InterpretaComando(void){
	uint16_t aux=0;
	switch(comando[0]){
	case 'T':
		if(comando[1]){
			aux= atoi(&comando[1]);
			ICR1= aux;
		}
		printf("T periodo=%u\r\n",ICR1);
		break;
	case 'D':
		if(comando[1]){
			aux = atoi(&comando[1]);
			uint32_t tmp1;
			tmp1= ((uint32_t)aux*(uint32_t)ICR1)/100;
			OCR1A= (uint16_t)tmp1;
		}
		printf("Duty cycle =%u\r\n",OCR1A);
		break;
	default:
		break;
	}
}
ISR(TIMER1_OVF_vect){
	static int i=0;
	if(++i==256){
		i=0;
	}
	OCR1A=sinus[i];
}
