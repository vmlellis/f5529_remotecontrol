/*
 * esc.c
 *
 * 	RELE - P8.1 / P8.2 / P3.3 / P3.4
 * 	ESC - P1.4 / P1.5 / P6.3 / P6.4
 *
 *  Created on: 25/04/2015
 *      Author: Victor
 */


#include "esc.h"

static unsigned int esc_ticks[4];
static volatile int esc_counter = 0;
static volatile int esc_totalWait = 0;

#define TRIM_DURATION 2  // compensation ticks to trim adjust for digitalWrite delays

/**
 * Inicializar os RELES e ESCs
 */
void esc_init(void) {
	/*
	 * Definir as portas para os RELES como saida e desligados (HIGH)
	 */
	RELE1SEL &= ~RELE1BIT;	// PINO RELE 1 - I/O
	RELE1DIR |= RELE1BIT; 	// PINO RELE 1 - SAIDA
	RELE1OUT |= RELE1BIT;	// PINO RELE 1 - ATIVADO

	RELE2SEL &= ~RELE2BIT;	// PINO RELE 2 - I/O
	RELE2DIR |= RELE2BIT; 	// PINO RELE 2 - SAIDA
	RELE2OUT |= RELE2BIT;	// PINO RELE 2 - ATIVADO

	RELE3SEL &= ~RELE3BIT;	// PINO RELE 3 - I/O
	RELE3DIR |= RELE3BIT; 	// PINO RELE 3 - SAIDA
	RELE3OUT |= RELE3BIT;	// PINO RELE 3 - ATIVADO

	RELE4SEL &= ~RELE4BIT;	// PINO RELE 4 - I/O
	RELE4DIR |= RELE4BIT; 	// PINO RELE 4 - SAIDA
	RELE4OUT |= RELE4BIT;	// PINO RELE 4 - ATIVADO

	ESC1SEL &= ~ESC1BIT;	// PINO ESC 1 - I/O
	ESC1DIR |= ESC1BIT;		// PINO ESC 1 - SAIDA

	ESC2SEL &= ~ESC2BIT;	// PINO ESC 2 - I/O
	ESC2DIR |= ESC2BIT;		// PINO ESC 2 - SAIDA

	ESC3SEL &= ~ESC3BIT;	// PINO ESC 3 - I/O
	ESC3DIR |= ESC3BIT;		// PINO ESC 3 - SAIDA

	ESC4SEL &= ~ESC4BIT;	// PINO ESC 4 - I/O
	ESC4DIR |= ESC4BIT;		// PINO ESC 4 - SAIDA

	int i;
	for(i = 0; i < 4; i++)
		esc_ticks[i] = usToTicks(DEFAULT_PULSE_WIDTH);

	esc_counter = -1;
	esc_totalWait = 0;

	// Enable Timer
	esc_timer(); // Habilita o primeiro ESC
	TA1CCTL0 = CCIE;                    	// CCR0 interrupt enabled
	TA1CTL = TASSEL_2 + MC_1 + ID_3;       	// prescale SMCLK/8, upmode
}

/**
 * Rotina do timer
 */
void esc_timer(void) {
	static unsigned long wait;
	if (esc_counter >= 0) {
		switch (esc_counter)
		{
			case 0:
				ESC1OUT &= ~ESC1BIT; // PINO ESC 1 - LOW
				break;
			case 1:
				ESC2OUT &= ~ESC2BIT; // PINO ESC 2 - LOW
				break;
			case 2:
				ESC3OUT &= ~ESC3BIT; // PINO ESC 3 - LOW
				break;
			case 3:
				ESC4OUT &= ~ESC4BIT; // PINO ESC 4 - LOW
				break;
		}
	}

	esc_counter++;

	if (esc_counter < 4) {
		switch (esc_counter) {
			case 0:
				ESC1OUT |= ESC1BIT; // PINO ESC 1 - HIGH
				break;
			case 1:
				ESC2OUT |= ESC2BIT; // PINO ESC 2 - HIGH
				break;
			case 2:
				ESC3OUT |= ESC3BIT; // PINO ESC 3 - HIGH
				break;
			case 3:
				ESC4OUT |= ESC4BIT; // PINO ESC 4 - HIGH
				break;
		}
		esc_totalWait += esc_ticks[esc_counter];
		TA1CCR0 = esc_ticks[esc_counter];
	}
	else {
		/* Wait for the remaining of REFRESH_INTERVAL. */
		wait = usToTicks(REFRESH_INTERVAL) - esc_totalWait;
		esc_totalWait = 0;
		TA1CCR0 = (wait < 1000 ? 1000 : wait);
		esc_counter = -1;
	}
}

/**
 * Conectar um rele
 */
void esc_connect(uint8_t num) {
	switch (num) {
		case 1:
			RELE1OUT &= ~RELE1BIT; // PINO RELE 1 - BAIXO
			break;
		case 2:
			RELE2OUT &= ~RELE2BIT; // PINO RELE 2 - BAIXO
			break;
		case 3:
			RELE3OUT &= ~RELE3BIT; // PINO RELE 3 - BAIXO
			break;
		case 4:
			RELE4OUT &= ~RELE4BIT; // PINO RELE 4 - BAIXO
			break;
		default:
			break;
	}
}

/**
 * Conectar todos os reles
 */
void esc_connectAll() {
	RELE1OUT &= ~RELE1BIT;
	RELE2OUT &= ~RELE2BIT;
	RELE3OUT &= ~RELE3BIT;
	RELE4OUT &= ~RELE4BIT;
}

/**
 * Desligar um rele
 */
void esc_disconnect(uint8_t num) {
	switch (num) {
		case 1:
			RELE1OUT |= RELE1BIT;	// PINO RELE 1 - ALTO
			break;
		case 2:
			RELE2OUT |= RELE2BIT;	// PINO RELE 2 - ALTO
			break;
		case 3:
			RELE3OUT |= RELE3BIT;	// PINO RELE 3 - ALTO
			break;
		case 4:
			RELE4OUT |= RELE4BIT;	// PINO RELE 4 - ALTO
			break;
		default:
			break;
	}
}

/**
 * Desliga todos os reles
 */
void esc_disconnectAll(void) {
	RELE1OUT |= RELE1BIT;
	RELE2OUT |= RELE2BIT;
	RELE3OUT |= RELE3BIT;
	RELE4OUT |= RELE4BIT;
}

/**
 * Atualiza o motor
 */
void updateMotor(int num, int value) {
	volatile int v = usToTicks(value);  // convert to ticks after compensating for interrupt overhead
	switch (num) {
		case 1:
			// Habilitar a configuracao do motor 1
			esc_ticks[0] = v; // this is atomic, no need to disable Interrupts
			break;
		case 2:
			// Habilitar a configuracao do motor 2
			esc_ticks[1] = v; // this is atomic, no need to disable Interrupts
			break;
		case 3:
			// Habilitar a configuracao do motor 3
			esc_ticks[2] = v; // this is atomic, no need to disable Interrupts
			break;
		case 4:
			// Habilitar a configuracao do motor 4
			esc_ticks[3] = v; // this is atomic, no need to disable Interrupts
			break;
		default:
			break;
	}
}

/**
 * Habilita a configuração para um motor
 * PS.: Não executar com o motor em movimento
 */
void esc_enableConfigMotor(uint8_t num) {
	int value = MAX_PULSE_ESC - TRIM_DURATION;   // Envia o mais alto sinal que o ESC pode receber
	updateMotor(num, value);
	/*volatile int v = usToTicks(value);  // convert to ticks after compensating for interrupt overhead
	switch (num) {
		case 1:
			// Habilitar a configuracao do motor 1
			esc_ticks[0] = v; // this is atomic, no need to disable Interrupts
			break;
		case 2:
			// Habilitar a configuracao do motor 2
			esc_ticks[1] = v; // this is atomic, no need to disable Interrupts
			break;
		case 3:
			// Habilitar a configuracao do motor 3
			esc_ticks[2] = v; // this is atomic, no need to disable Interrupts
			break;
		case 4:
			// Habilitar a configuracao do motor 4
			esc_ticks[3] = v; // this is atomic, no need to disable Interrupts
			break;
		default:
			break;
	}*/
}

/**
 * Habilita a configuração para um motor
 * PS.: Executar após o 'esc_enableConfigMotor'
 */
void esc_activeConfigMotor(uint8_t num) {
	int value = MIN_PULSE_ESC - TRIM_DURATION;
	updateMotor(num, value);
	/*volatile int v = usToTicks(value);
	switch (num) {
		case 1:
			// Ativa a configuracao do motor 1
			esc_ticks[0] = v;
			break;
		case 2:
			// Ativa a configuracao do motor 2
			esc_ticks[1] = v;
			break;
		case 3:
			// Ativa a configuracao do motor 3
			esc_ticks[2] = v;
			break;
		case 4:
			// Ativa a configuracao do motor 4
			esc_ticks[3] = v;
			break;
		default:
			break;
	}*/
}

void esc_setDefaultPulse(uint8_t num) {
	int value = DEFAULT_PULSE_WIDTH;
	updateMotor(num, value);
	/*volatile int v = usToTicks(value);
	switch (num) {
		case 1:
			// Ativa a configuracao do motor 1
			esc_ticks[0] = v;
			break;
		case 2:
			// Ativa a configuracao do motor 2
			esc_ticks[1] = v;
			break;
		case 3:
			// Ativa a configuracao do motor 3
			esc_ticks[2] = v;
			break;
		case 4:
			// Ativa a configuracao do motor 4
			esc_ticks[3] = v;
			break;
		default:
			break;
	}*/
}

/**
 *  Start do motor com velocidade minima
 */
void esc_startMotor(uint8_t num) {
	int value = MIN_FUNC_MOTOR - TRIM_DURATION;
	updateMotor(num, value);
	/*volatile int v = usToTicks(value);
	switch (num) {
		case 1:
			// Startar o motor 1
			esc_ticks[0] = v;
			break;
		case 2:
			// Startar o motor 2
			esc_ticks[1] = v;
			break;
		case 3:
			// Startar o motor 3
			esc_ticks[2] = v;
			break;
		case 4:
			// Startar o motor 4
			esc_ticks[3] = v;
			break;
		default:
			break;
	}*/
}

/**
 * Startar todos os motores
 */
void esc_startAllMotors() {
	// TODO: Startar todos os motores
	int value = MIN_FUNC_MOTOR - TRIM_DURATION;
	volatile int v = usToTicks(value);
	int i;
	for(i = 0; i < 4; i++) {
		esc_ticks[i] = v;
	}
}

/**
 * Mapear os valor
 */
uint32_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * Definir a velocidade do motor
 */
void esc_setSpeedMotor(uint8_t num, uint32_t value) {
	if (value > 1000)
		value = 1000;

	value = map(value, 0, 1000, MIN_FUNC_MOTOR, MAX_FUNC_MOTOR);
	value = value - TRIM_DURATION;
	updateMotor(num, value);

	/*
	volatile int v = usToTicks(value);

	switch (num) {
		case 1:
			// Definir velocidade do motor 1
			esc_ticks[0] = v;
			break;
		case 2:
			// Definir velocidade do motor 2
			esc_ticks[1] = v;
			break;
		case 3:
			// Definir velocidade do motor 3
			esc_ticks[2] = v;
			break;
		case 4:
			// Definir velocidade do motor 4
			esc_ticks[3] = v;
			break;
		default:
			break;
	}
	*/
}


/**
 * Parar o motor
 */
void esc_stopMotor(uint8_t num) {
	int value = STOP_FUNC_MOTOR - TRIM_DURATION;
	updateMotor(num, value);
	/*
	volatile int v = usToTicks(value);
	switch (num) {
		case 1:
			// Parar o motor 1
			esc_ticks[0] = v;
			break;
		case 2:
			// Parar o motor 2
			esc_ticks[1] = v;
			break;
		case 3:
			// Parar o motor 3
			esc_ticks[2] = v;
			break;
		case 4:
			// Parar o motor 4
			esc_ticks[3] = v;
			break;
		default:
			break;
	}*/
}

/**
 * Parar todos os motores
 */
void esc_stopAllMotors(void) {
	// TODO: Parar todos os motores
	int value = STOP_FUNC_MOTOR - TRIM_DURATION;
	volatile int v = usToTicks(value);
	int i;
	for(i = 0; i < 4; i++) {
		esc_ticks[i] = v;
	}
}
