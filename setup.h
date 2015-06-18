/*****************************************************************************
 * config.h
 *
 * Arquivo para a configuração
 *
 *  Criado em: 28/02/2015
 *      Autor: Victor
 *
 *  Construido com a IDE CCSv6 e compilador GNU v4.9.1 (Red Hat)
 *****************************************************************************/

#ifndef CONFIG_H_
#define CONFIG_H_

#include <msp430.h>
#include <inttypes.h>

#define F_CPU 25000000L   	// Frequencia da CPU: 25 MHz
#define BAUD_RATE 115200L	// Baud Rate do UART: 115200
#define I2C_FREQ 400000L  	// Frequencia do I2C: 400 kHz
#define SPI_CLOCK_SPEED 16000000L  // Frequencia do SPI: 16MHz

#define SPI_CLOCK_DIV() ((F_CPU / SPI_CLOCK_SPEED) + (F_CPU % SPI_CLOCK_SPEED == 0 ? 0:1))

#define WLAN_CS_PIN        BIT2  // P2.2
#define WLAN_EN_PIN        BIT5	 // P6.5
#define WLAN_IRQ_PIN       BIT0  // P2.0

#define SPI_SEL		P3SEL
#define SPI_DIR     P3DIR
#define SPI_OUT     P3OUT
#define SPI_REN     P3REN

#define SPI_MOSI 	BIT0 // P3.0
#define SPI_MISO 	BIT1 // P3.1
#define SPI_CLK 	BIT2 // P3.2

// Ports
#define WLAN_CS_SEL       	P2SEL
#define WLAN_CS_OUT       	P2OUT
#define WLAN_CS_DIR       	P2DIR

#define WLAN_EN_DIR      	P6DIR
#define WLAN_EN_OUT       	P6OUT

#define WLAN_IRQ_DIR    	P2DIR
#define WLAN_IRQ_IN        	P2IN
#define WLAN_IRQ_IES       	P2IES
#define WLAN_IRQ_IE        	P2IE
#define WLAN_IFG_PORT      	P2IFG

/**
 * Definições dos botões
 */

// Botão 0 - P2.1
#define BUTTON0_PIN BIT1
#define BUTTON0_DIR P2DIR
#define BUTTON0_OUT P2OUT
#define BUTTON0_REN P2REN
#define BUTTON0_IES P2IES
#define BUTTON0_IE  P2IE
#define BUTTON0_IFG P2IFG

// Botão 1 - P1.2
#define BUTTON1_PIN BIT2
#define BUTTON1_DIR P1DIR
#define BUTTON1_OUT P1OUT
#define BUTTON1_REN P1REN
#define BUTTON1_IES P1IES
#define BUTTON1_IE  P1IE
#define BUTTON1_IFG P1IFG

// Botão 2 - P1.3
#define BUTTON2_PIN BIT3
#define BUTTON2_DIR P1DIR
#define BUTTON2_OUT P1OUT
#define BUTTON2_REN P1REN
#define BUTTON2_IES P1IES
#define BUTTON2_IE  P1IE
#define BUTTON2_IFG P1IFG

// Botão 3 - P1.4
#define BUTTON3_PIN BIT4
#define BUTTON3_DIR P1DIR
#define BUTTON3_OUT P1OUT
#define BUTTON3_REN P1REN
#define BUTTON3_IES P1IES
#define BUTTON3_IE  P1IE
#define BUTTON3_IFG P1IFG

// Botão 4 - P1.5
#define BUTTON4_PIN BIT5
#define BUTTON4_DIR P1DIR
#define BUTTON4_OUT P1OUT
#define BUTTON4_REN P1REN
#define BUTTON4_IES P1IES
#define BUTTON4_IE  P1IE
#define BUTTON4_IFG P1IFG

/**
 * Definições para o BUZZER - P6.6
 */
#define BUZZER_PIN BIT6
#define BUZZER_DIR P6DIR
#define BUZZER_OUT P6OUT


/*
 * Definições para controle do clock
 */
#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )

/*
 * Definições para o WatchDog
 */
#define TICKS_PER_WDT_OVERFLOW 8192

// the whole number of microseconds per WDT overflow
#define MICROSECONDS_PER_WDT_OVERFLOW (clockCyclesToMicroseconds(TICKS_PER_WDT_OVERFLOW))

// o número em milissegundos para o estouro do WDT
#define MILLIS_INC (MICROSECONDS_PER_WDT_OVERFLOW / 1000)

// numero fracional de milisegundos para o overflow de WDT
#define FRACT_INC (MICROSECONDS_PER_WDT_OVERFLOW % 1000)
#define FRACT_MAX 1000

/**
 * Definições matematicas
 */
#define PI 3.1415926535897932384626433832795

void disableWatchDog(void);
void enableWatchDog(void);
void saveUsbPower(void);
void initClocks(void);
void setupUart(void);
void setupI2C(void);
void delay(uint32_t);
void delayMicroseconds(uint32_t);
uint8_t read_bits(uint8_t, uint8_t, uint8_t);
uint8_t write_bits(uint8_t, uint8_t, uint8_t);
unsigned long micros();
unsigned long millis();

#endif /* CONFIG_H_ */
