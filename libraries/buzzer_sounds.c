/*
 * buzzer_sounds.c
 *
 *  Created on: 10/06/2015
 *      Author: Victor
 *
 *  Referencias:
 *  - http://processors.wiki.ti.com/index.php/Playing_The_Imperial_March
 *  - http://www.princetronics.com/supermariothemesong/
 *  - http://www.instructables.com/id/Play-the-French-Can-Can-Using-an-Arduino-and-Buzze/step3/Figure-Out-Hardware-Requirements/
 *
 */

#include "buzzer_sounds.h"
#include "../setup.h"

//Definition of the notes' frequecies in Hertz.
#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978

#define BUZZER_c 261
#define BUZZER_d 294
#define BUZZER_e 329
#define BUZZER_f 349
#define BUZZER_g 391
#define BUZZER_gS 415
#define BUZZER_a 440
#define BUZZER_aS 455
#define BUZZER_b 466
#define BUZZER_cH 523
#define BUZZER_cSH 554
#define BUZZER_dH 587
#define BUZZER_dSH 622
#define BUZZER_eH 659
#define BUZZER_fH 698
#define BUZZER_fSH 740
#define BUZZER_gH 784
#define BUZZER_gSH 830
#define BUZZER_aH 880


/* play tone */
void playTone(int _tone, int duration) {
	long i;
    for(i = 0; i < duration * 1000L; i += _tone*2 ) {
    	BUZZER_OUT |= BUZZER_PIN;
        delayMicroseconds(_tone);
        BUZZER_OUT &= ~BUZZER_PIN;
        delayMicroseconds(_tone);
    }
}


/* play notes */
void playNote(char note, int duration) {
    char name[] = {'c', 'd', 'e', 'f', 'g', 'a', 'b', 'c'};
    int _tone[] = {1915, 1700, 1519, 1432, 1275, 1136, 1014, 956};
    /* play the tone corresponding to the tone name */
    int i;
    for(i = 0; i < 8; i++) {
        if(name[i] == note) {
            playTone(_tone[i], duration);
        }
    }
}

void playTest() {
	int length = 10;         /* the number of notes */
	char notes[] = "ccggaaffeeddc ";    /* the notes */
	int beats[] = { 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 2, 4 };  /* the beats*/
	int tempo = 300;    /* the tempo */

	int i;
	for(i = 0; i < length; i++) {
		if(notes[i] == ' ') {
			delay(beats[i] * tempo);
		} else {
			playNote(notes[i], beats[i] * tempo);
		}
		delay(tempo / 2);    /* delay between notes */
	}
}


//melodia do MARIO THEME
int melodia[] = {660,660,660,510,660,770,380,510,380,320,440,480,450,430,380,660,760,860,700,760,660,520,580,480,510,380,320,440,480,450,430,380,660,760,860,700,760,660,520,580,480,500,760,720,680,620,650,380,430,500,430,500,570,500,760,720,680,620,650,1020,1020,1020,380,500,760,720,680,620,650,380,430,500,430,500,570,585,550,500,380,500,500,500,500,760,720,680,620,650,380,430,500,430,500,570,500,760,720,680,620,650,1020,1020,1020,380,500,760,720,680,620,650,380,430,500,430,500,570,585,550,500,380,500,500,500,500,500,500,500,580,660,500,430,380,500,500,500,500,580,660,870,760,500,500,500,500,580,660,500,430,380,660,660,660,510,660,770,380};

//duraçao de cada nota
int duracaodasnotas[] = {100,100,100,100,100,100,100,100,100,100,100,80,100,100,100,80,50,100,80,50,80,80,80,80,100,100,100,100,80,100,100,100,80,50,100,80,50,80,80,80,80,100,100,100,100,150,150,100,100,100,100,100,100,100,100,100,100,150,200,80,80,80,100,100,100,100,100,150,150,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,150,150,100,100,100,100,100,100,100,100,100,100,150,200,80,80,80,100,100,100,100,100,150,150,100,100,100,100,100,100,100,100,100,100,100,100,100,60,80,60,80,80,80,80,80,80,60,80,60,80,80,80,80,80,60,80,60,80,80,80,80,80,80,100,100,100,100,100,100,100};

// Pausa depois da nota
int pausadepoisdasnotas[] ={150,300,300,100,300,550,575,450,400,500,300,330,150,300,200,200,150,300,150,350,300,150,150,500,450,400,500,300,330,150,300,200,200,150,300,150,350,300,150,150,500,300,100,150,150,300,300,150,150,300,150,100,220,300,100,150,150,300,300,300,150,300,300,300,100,150,150,300,300,150,150,300,150,100,420,450,420,360,300,300,150,300,300,100,150,150,300,300,150,150,300,150,100,220,300,100,150,150,300,300,300,150,300,300,300,100,150,150,300,300,150,150,300,150,100,420,450,420,360,300,300,150,300,150,300,350,150,350,150,300,150,600,150,300,350,150,150,550,325,600,150,300,350,150,350,150,300,150,600,150,300,300,100,300,550,575};
int pausa = 0;

extern volatile int buzzer_counter_ms;

//Mario main theme melody
uint8_t playMarioSong(int *nota) {

	//for para tocar as 156 notas começando no 0 ate 156 ++ incrementado
	/*int nota;
	for (nota = 0; nota < 156; nota++) {
		int duracaodanota = duracaodasnotas[nota];
		playTone(melodia[nota],duracaodanota);
		delay(pausadepoisdasnotas[nota]*0.7);
	}*/

	if (buzzer_counter_ms >= pausa) {
		int duracaodanota = duracaodasnotas[*nota];
		playTone(melodia[*nota],duracaodanota);

		pausa = pausadepoisdasnotas[*nota]*0.7;
		//delay(pausadepoisdasnotas[*nota]*0.7);
		(*nota)++;
		buzzer_counter_ms = 0;
	}

	if (*nota == 156) {
		return 1;
	}

	return 0;
}

void resetPausa() {
	pausa = 0;
	buzzer_counter_ms = 0;
}
