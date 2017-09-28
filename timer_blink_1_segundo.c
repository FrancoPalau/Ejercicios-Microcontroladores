/* Realizar Blink de un led de 1 segundo con Timer 1, en una ISR*/
/******************************************************************/
#define F_CPU 16000000

#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>

ISR(TIMER1_COMPA_vect); //Rutina de interrupcion

int main() {
    DDRB=(1<<DDB5);//Habilita salida

    OCR1A=62499;
	//valor de comparacion(62500-1) para que de 1 segundo(1 segundo esta el led encendido y luego
	// 1 segundo esta apagado, para que dure 1 segundo el periodo total seria 62499/2)

    TCCR1B=(1<<CS12)|(1<<WGM12); //seteamos el prescaler en 256 y el modo CTC

    TIFR1&=~(1<<OCF1A); // borra flag de compare match A

    TIMSK1=(1<<OCIE1A); // habilita interrupcion por Match A

    sei(); //activa interrupciones globales

    while(1){
        /*PORTB^=(1<<PORTB4);
        _delay_ms(1000);*/
    }

    return 0;
}

ISR(TIMER1_COMPA_vect){  //funcion de interrupcion
    PORTB^=(1<<PORTB5); //toggle del PORTB5
}
