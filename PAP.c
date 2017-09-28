/*
 *
 *
 *
 * Author : Palau Franco
 */
#define	F_CPU 16000000

#define brate0 19200

#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>

unsigned int Tpos,Thom,P,PA,F,B,varibleEstado;

#define ACTIVADO 1
#define DESACTIVADO 0
#define HOMACT 2
#define POSACT 3
#define POSOK 4

#define TMIN 1
#define TMAX 99
#define PMIN 0
#define PMAX 9999

uint8_t indcom;				// índice para llenar el buffer de recepción
char comando[30];			// buffer de recepción
volatile uint8_t registroEstado;
volatile int motorPap;

void InterpretaComando(void);	// Función para interpretar comando que llega por puerto serie
void mi_UART_Init( unsigned int);
int mi_putc(char, FILE *stream);		// Función para transmitir caracter por UART
int mi_getc(FILE *stream);				// Función para recibir caracter por UART
void avanza(void);
void retrocede(void);
void homing(void);

// Las 2 definiciones siguientes son para usar getc y putc para E/S de caracteres por UART
#define getc() mi_getc(&uart_io)		// redefine la primitiva de entrada como  función recibir por UART
#define putc(x) mi_putc(x,&uart_io)		// redefine la primitiva de salida como función transmitir por UART

FILE uart_io = FDEV_SETUP_STREAM(mi_putc, mi_getc, _FDEV_SETUP_RW); // Declara un tipo stream de E/S

//enum tipoEstado{activado, desactivado}motorPap;             //Estado del motor

enum tipoEstadoHoming{encendido,apagado}estadoHoming;   //Estado del homing

enum tipoEstadoEje{referenciado, noReferenciado}Eje; //Estado del Eje

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
	case 'F':
		switch (comando[1])
		{
		case 'T':
		    switch(comando[2])
			{
			case 'P':
				if(comando[3]){
					aux = atoi(&comando[3]);
					if (aux>=TMIN && aux<=TMAX){
						Tpos=aux*100;//aca se convierte a milisegundos
					}
				}
				printf(":FTP%d\r\n",(Tpos/100)); //aca se imprime en decimas de segundo
				break;
			case 'H':
				if(comando[3]){
					aux = atoi(&comando[3]);
					if (aux>=TMIN && aux<=TMAX){
						Thom=aux*100;//se convierte a milisegundos
					}
				}
				printf(":FTH%d\r\n",(Thom/100)); //aca se imprime en decimas de segundo
				break;
			default:
				break
			}
			break;
		case 'P':
			if (comando[2]){
				aux = atoi(&comando[2]);
				if (aux<=PMAX && aux>=PMIN ) {
					P=aux;
				}
			}
			printf(":FP%d\r\n",P);
			break;
		case 'A':                            //Activar o desactivar el pap
			if(comando[2]=='1'){
				PORTB&=~(1<<PORTB2);                   //prende el enable
				motorPap=ACTIVADO;
				varibleEstado=ACTIVADO;
				printf(":FA%d\n",1 );             //1 en el primer bit del registroEstado
			}else if(comando[2]=='0'){
				PORTB|=(1<<PORTB2);                     //prende el enable
				motorPap=DESACTIVADO;
				varibleEstado=DESACTIVADO;
				printf(":FA%d\n",0 );            //0 en el primer bit del registroEstado
			}else{
				printf(":FA%d\n",motorPap );
			}
			break;
		case 'h':
			estadoHoming=encendido;
			break;
		case 'E':
			if(varibleEstado==ACTIVADO){
				printf(":F%s\r\n","A" );
			}else if(varibleEstado==DESACTIVADO){
				printf(":F%s\n","D" );
			}else if(varibleEstado==HOMACT){
				printf(":F%s\r\n","h" );
			}else if(varibleEstado==POSACT){
				printf(":F%s\r\n","p" );
			}else if(varibleEstado==POSOK){
				printf(":F%s\r\n","P" );
			}else{
				//do nothing
			}
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

void homing(void){
	printf(":Fh\r\n");
	while((PINB&(1<<PORTB3))){
		//avanza();                        //Estos dos hay que cambiarlos
		retrocedeHoming();                       //de acuerdo a la maqueta probada
	}
	_delay_ms(100);
	T=1000;//periodo de avance de 1 segundo
	while (!(PINB&(1<<PORTB3))) {
		//retrocede();
		avanza();
	}
	estadoHoming=apagado;
	printf(":FH\r\n");
	Eje=referenciado;
	PA=0;
	P=0;
	varibleEstado=POSOK;
}
void retrocedeHoming(void){
	PORTB &= ~(1<<PORTB4);
	PORTB |= (1<<PORTB5);        // enciende PORTB5
	mi_delay(Thom/2);
	PORTB &= ~(1<<PORTB5);       // apaga PORTB5
	mi_delay(Thom/2);
	PA=PA-B;
}
void avanza(void){
	PORTB |=(1<<PORTB4);
	PORTB |= (1<<PORTB5);        // enciende PORTB5
	mi_delay(T/2);
	PORTB &= ~(1<<PORTB5);       // apaga PORTB5
	mi_delay(T/2);
	PA=PA+F;
}
void retrocede(void){
	PORTB &= ~(1<<PORTB4);
	PORTB |= (1<<PORTB5);        // enciende PORTB5
	mi_delay(T/2);
	PORTB &= ~(1<<PORTB5);       // apaga PORTB5
	mi_delay(T/2);
	PA=PA-B;
}
//---------------------------------------------------------------------------------------------
//		Rutina de servicio de interrupción por recepción de datos en UART
//      Espera mensajes del tipo [:][comando][Parámetro][\r] y lo guarda en el buffer Comando[]
//      limpiando los delimitadores y agregando un \0 al parámetro numérico para poder usar atoi()
//      Por ejemplo ":T342\r"
//      Al recibir ':' (delimitador de inicio) pone el índice indcom=0. No guarda ':'
//      'T','3','4','2' los guarda en posiciones sucesivas del buffer Comando[] (sin evaluarlos) incrementando indcom
//		Al recibir '\r' (delimitador de final) completa colocando un NULL (\0) en la posición
//      de Comando[] apuntada por indcom (es decir Comando[] queda "T342\0..." y llama a Interpretacomando()

ISR(USART_RX_vect)
{
	char dato;
	dato=getc();
	switch(dato)
	{
		case '/':
		case ':':				// Delimitador de inicio
			indcom=0;				// Inicializa índice de buffer de recepción
			break;
		case '\r':
		case ';':				// Delimitador de final
			comando[indcom]=0;	// coloca \0 luego del último caracter recibido antes de \r
			InterpretaComando();// Llama a función intérprete de comandos
			break;
		default:				// Todo lo que está entre delimitadores, Ej. 'T','3','4','2'
			comando[indcom++]=dato; // Guarda en elemento del buffer e incrementa indcom para apuntar a siguiente
			break;
	}
}

void mi_delay(int ms)			// Delay con argumento variable.
{								// Es inexacto (hay retardos adicionales en el lazo 'for')
	int t;
	for(t=0;t<ms;t++)
		_delay_ms(1);
}

int main(void)
{
	mi_UART_Init(brate0);
	stdout = stdin = &uart_io;  // El stream (FILE) uart_io es la E/S estándar, es decir para putc y getc

	DDRB = (1<<DDB4)|(1<<DDB5)|(1<<DDB2)|(1<<DDB1);				//12(dir),13(pulso),10(enable),9(modo) salidas

	//Inicializacion del resto de las variables
	motorPap=DESACTIVADO;
	PORTB|=(1<<PORTB2);
	Eje=noReferenciado;
	estadoHoming=apagado;
	Tpos=2*100;
	Thom=5*100;
	PA=0;
	P=0;
	F=1;
	B=1;

	printf("GLOBAL\r\n");
	indcom=0;
	UCSR0B|= (1<<RXCIE0);	// Interrupcion Rx UART0
	sei();					// Interrupcion global

	while(1){
		switch (motorPap) {
		case ACTIVADO:
			switch (Eje) {
			case referenciado:
				if (P>PA){
					varibleEstado=POSACT;
					avanza();
				}
				else if (P<PA){
					varibleEstado=POSACT;
					retrocede();
				}
				else {
					varibleEstado=POSOK;
				}
				break;
			case noReferenciado:
				if (estadoHoming==encendido) {
					homing();
				}
				break;
			}
		case DESACTIVADO:
			//do nothing
			break;
		default:
			break;
		}
	}
}





/*case 'F':                         //avance en pasos
	if(comando[1]){
		aux = atoi(&comando[1]);
		F=aux;
	}
	printf("F:%d\r\n",F);
	break;
case 'B':                        //retroceso en pasos
	if(comando[1]){
		aux = atoi(&comando[1]);
		B=aux;
	}
	printf("B:%d\r\n",B);
	break;
case 'M':
	if(comando[1]=='1'){
		PORTB|=(1<<PORTB1);//medio paso
		registroEstado|=(1<<H_F);       //1 en el quinto bit de registroEstado
	}else if(comando[1]=='0'){
		PORTB&=~(1<<PORTB1);//paso completo
		registroEstado&=~(1<<H_F);      //0 en el quinto bit de registroEstado
	}
	break;*/
