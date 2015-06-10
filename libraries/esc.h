/*
 * esc.h
 *
 *  Created on: 25/04/2015
 *      Author: Victor
 */

#ifndef LIBRARIES_ESC_H_
#define LIBRARIES_ESC_H_

#include <inttypes.h>
#include "../setup.h"

/* Definições de Servo (Retirado da biblioteca de Servo da Energia) */
#define MIN_PULSE_WIDTH       544     // the shortest pulse sent to a servo [uS]
#define MAX_PULSE_WIDTH      2400     // the longest pulse sent to a servo [uS]
#define DEFAULT_PULSE_WIDTH  1500     // default pulse width when servo is attached
#define REFRESH_INTERVAL    20000     // servos refresh period in microseconds

#define usToTicks(_us)    (( clockCyclesPerMicrosecond()* _us) / 8)     // converts microseconds to timer ticks (assumes prescale of 8)
#define ticksToUs(_ticks) (( (unsigned)_ticks * 8)/ clockCyclesPerMicrosecond() ) // converts from ticks back to microseconds

/* Definições próprias */
#define STOP_FUNC_MOTOR 725			// Pulso para parada do motor
#define MIN_FUNC_MOTOR 800			// Pulso minimo para o arranque do motor
#define MAX_FUNC_MOTOR 1800			// Pulso máximo para o funcionamento do motor (Não definido o máximo do motor para evitar desgaste)
#define MAX_PULSE_ESC 2000			// Pulso maximo para o ESC
#define MIN_PULSE_ESC 700			// Pulso minimo para o ESC

/* Definição para o RELE 1 - P8.1 */
#define RELE1SEL P8SEL
#define RELE1DIR P8DIR
#define RELE1OUT P8OUT
#define RELE1BIT BIT1

/* Definição para o RELE 2 - P8.2 */
#define RELE2SEL P8SEL
#define RELE2DIR P8DIR
#define RELE2OUT P8OUT
#define RELE2BIT BIT2

/* Definição para o RELE 3 - P1.6 */
#define RELE3SEL P1SEL
#define RELE3DIR P1DIR
#define RELE3OUT P1OUT
#define RELE3BIT BIT6

/* Definição para o RELE 4 - P2.7 */
#define RELE4SEL P2SEL
#define RELE4DIR P2DIR
#define RELE4OUT P2OUT
#define RELE4BIT BIT7

/* Definição para o ESC 1 - P6.4 */
#define ESC1SEL P6SEL
#define ESC1DIR P6DIR
#define ESC1OUT P6OUT
#define ESC1BIT BIT4

/* Definição para o ESC 2 - P7.0 */
#define ESC2SEL P7SEL
#define ESC2DIR P7DIR
#define ESC2OUT P7OUT
#define ESC2BIT BIT0

/* Definição para o ESC 3 - P3.6 */
#define ESC3SEL P3SEL
#define ESC3DIR P3DIR
#define ESC3OUT P3OUT
#define ESC3BIT BIT6

/* Definição para o ESC 4 - P3.5 */
#define ESC4SEL P3SEL
#define ESC4DIR P3DIR
#define ESC4OUT P3OUT
#define ESC4BIT BIT5

/* Inicialização */
void esc_init();
void esc_timer();

/* RELE */
void esc_connect(uint8_t);
void esc_connectAll();
void esc_disconnect(uint8_t);
void esc_disconnectAll();

/* Para configuração do motor */
void esc_enableConfigMotor(uint8_t);
void esc_activeConfigMotor(uint8_t);

/* Controle dos motores */
void esc_startMotor(uint8_t);
void esc_startAllMotors();
void esc_setSpeedMotor(uint8_t, uint32_t);
void esc_stopMotor(uint8_t);
void esc_stopAllMotors();



#endif /* LIBRARIES_ESC_H_ */
