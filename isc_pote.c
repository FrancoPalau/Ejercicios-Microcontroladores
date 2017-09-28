/*
 * EJ01_SPI_MCP4821.c
 * Manejo de un DAC MCP4821 por interfaz SPI
 * 1) Hacer pines /CS=0, /LDAC=1
 * 2) Transmitir primer byte: A/B BUF GA SHDN D11 D10 D9 D8.        En MCP4821 Ej. 0 1 1 1 D11 D10 D9 D8
 * 3) Transmitir segundo byte: D7 D6 D5 D4 D3 D2 D1 D0
 * 4) Hacer pin /CS=1
 * 5) Generar pulso invertido en /LDAC
 * Created: 25/05/2017 10:48:07 p.m.
 * Author : MyEP
 */
#define F_CPU 16000000

#include <avr/io.h>			// definición de registros
#include <stdint.h>			// tipos enteros
#include <util/delay.h>		// para delays
#include "SPI_master.h"		// biblioteca creada a partir de código de ejemplo de datasheet

#define pinCS PORTB2		// pin CS de MCP4821
#define pinLDA PORTB1		// pin LDA de MCP4821
#define control	0b00010001	// b7 a b4 son los 4 bits de control del DAC. A/B BUF GA SHDN x x x x
#define modoSPI 0

// Función para escribir el valor (0 a 4095) en el DAC MCP4821
void EscribeDAC(uint16_t valor)
{
	uint8_t aux;
	PORTB&=~(1<<pinCS);				// Habilita MCP4821
	//aux=control|((valor&0xF00)>>8); // 4 bits de control y b11 a b8 de valor
	SPI_transfer(control);				// transmite por SPI
	aux=(uint8_t)(valor&0xFF);		// b7 a b0 de valor
	SPI_transfer(aux);				// transmite por SPI
	_delay_us(1);
	PORTB|=(1<<pinCS);				// deshabilita MCP4821
	_delay_us(1);				//
}

int main(void)
{
	uint16_t i;					   // Variable para generar rampa numérica (para probar en DAC
	SPI_init(0,0);				   // inicializa en modo 0, con prescaler x4 (SCK=16/4=4Mbps)

	PORTB|=(1<<pinCS); // Pines de /CS y /LDA del MCP4821. Pone en alto
	DDRB|=(1<<pinCS);  // Pines de /CS y /LDA del MCP4821. Los define como salidas
    while (1)
    {
		for(i=0;i<256;i++)		   // Rampa numérica, genera rampa de voltaje en salida de MCP4821
		{
			EscribeDAC(i);
			_delay_ms(100);
		}

    }
}
