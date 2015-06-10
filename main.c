/*
 * main.c
 *
 *  Construido com a IDE CCSv6 e compilador GNU v4.9.1 (Red Hat)
 */


#include <msp430f5529.h>
#include "setup.h"
#include "uart/uart_tx.h"
#include "i2c/lcd/lcd.h"
#include "i2c/lcd/lcd_blue.h"
#include "spi/cc3000/WiFi.h"
#include "spi/cc3000/WiFiUDP.h"
#include "libraries/utils.h"
#include "libraries/esc.h"

#define UDP_PORT 2390



/*volatile int counter = 0;
volatile int counterRX = 0;
volatile int counterTX = 0;
volatile int counterSPI = 0;*/

void initLeds(void);
void initBuzzer(void);
void initButtons(void);
void setupTimers(void);

// Para sair do modo de baixo consumo no modo I2C
static volatile uint8_t FLAG_wakeUpI2C = 0;

/* Metodos externos para o I2C */
//extern void i2c_nack(void);
extern void i2c_rx(void);
extern void i2c_tx(void);
//extern void i2c_state_isr(void);
//extern void i2c_txrx_isr(void);

/* Metodo externo para o UART RX */
extern void uart_rx(void);

/* Variaveis WDT */
volatile unsigned long wdt_overflow_count = 0;
volatile unsigned long wdt_millis = 0;
volatile unsigned int wdt_fract = 0;

/* Para o controle do sleep */
volatile uint8_t sleeping = 0;

// Incrementa quando sleeping.
uint16_t SMILLIS_INC = 0;
uint16_t SFRACT_INC = 0;

// Informa se deve ser realizado a leitura dos sensores
volatile uint8_t readSensors = 0;

// Parametros do magnometro (HMC5884L)
uint8_t magEnabled = 0;			// Informa que o magnometro estah habilitado
int mx = 0, my = 0, mz = 0;		// Dados do magnometro

// Parametros do barometro (BMP085)
uint8_t barEnabled = 0;
long temp10 = 0, pressure = 0;
float temp = 0.0, altitude = 0.0;

// Parametros do accel/gyro (MPU6050)
uint8_t mpuEnabled = 0;
int16_t mpuTemp = 0;
float mpuTempDegrees = 0.0;
int16_t gx = 0, gy = 0, gz = 0;	// Dados do giroscopio
int16_t ax = 0, ay = 0, az = 0; // Dados do acelerometro


// LCD
uint8_t lcdEnabled = 0;
void clearLcdLine(int line);

// WiFi
extern void IntSpiGPIOHandler(void);
uint8_t wifiInit = 0;
uint8_t wifiConnected = 0;
volatile uint8_t udpInitialized = 0;
unsigned int timeoutWifi = 30000;   // Milliseconds
uint8_t wiFiServerInitialized = 0;
char packetBuffer[255];
char replyBuffer[100];
/*char replyBuffer[] = "acknowledged";
char replyBuffer2[] = "teste ok";*/
void showIPAddress(void);
void showRemoteAddress(uint32_t);
void showSSID(void);
uint8_t connectWiFi(void);
uint8_t connectWiFiMemory(void);
void printFirmwareWifiVersion(void);
volatile int packetSize;

const char *CMD_STRINGS[] = {"one","two","three"};


// Prototipos
//char* append(const char *s, char c);

uint8_t endsWith(char *str, char *suffix);
uint8_t startsWith(char *str, char *pre);
void printIndex();

//char current_line[100] = "";
char lcd_line[21] = "";


volatile uint8_t readUDP = 0;
volatile uint8_t lcdCMD = 0;
void readUDPBuffer(void);
void replyUDP(void);

uint8_t smiley[8] = {
  0b00000,
  0b10001,
  0b00000,
  0b00000,
  0b10001,
  0b01110,
  0b00000,
};

/*
 * main.c
 */
int main(void) {
	//unsigned int i = 0;

	initLeds();
	initBuzzer();
	initButtons();
    disableWatchDog();
    initClocks();
    enableWatchDog();
    saveUsbPower();
    setupUart();

    uart_printf("UART inicializado!\r\n");

    setupI2C();

    setupTimers();

    magEnabled = 0;
    barEnabled = 0;
    mpuEnabled = 0;
    lcdEnabled = 0;

    lcdCMD = 0;

    lcdEnabled = lcd_blue_detect();
    if (lcdEnabled) {
    	uart_printf("LCD habilitado!\r\n");
    	lcd_blue_config();
    	lcd_clear();
    	lcd_setCursor(0,1);
    	//strcpy(lcd_line, "Inicializando...");
    	lcd_print("Inicializando...");
    	lcd_createChar(0, smiley);
    }
    else {
    	uart_printf("Erro na Inicialização do LCD!\r\n");
    }


    //uart_printf("Inicializando o WiFi...\r\n");
    _enable_interrupts();
    if (WiFi_init()) {
    	//uart_printf("WiFi inicializado!\r\n");
    	wifiInit = 1;
    	//WiFiUDP_init();

    	//printFirmwareWifiVersion();
    }
    else {
    	uart_printf("Erro na Inicialização do WiFi!\r\n");
    	if (lcdEnabled) {
    		lcd_setCursor(0,1);
    		//strcpy(lcd_line, "Erro: Modulo WiFi");
    		lcd_print("Erro: Modulo WiFi");
    	}
    }




    if (wifiInit) {
		wifiConnected = connectWiFiMemory();
		if (wifiConnected) {
			uart_printf("WiFi conectado!\r\n");
			showSSID();
			showIPAddress();

			WiFiUDP_init();
			if(WiFiUDP_start(UDP_PORT)) {

				uart_printf("UDP inicializado!\r\n");

				lcd_setCursor(0,0);
				lcd_write(0);
				lcd_print(" Iniciado!");

				udpInitialized = 1;
			}

		}
		else {
			if (lcdEnabled) {
				lcd_setCursor(0,1);
				//strcpy(lcd_line, "Erro: Nao Conectado");
				lcd_print("Erro: Nao Conectado");
			}
		}
    }


    while (1) {

    	if (lcdCMD > 0) {
    		switch(lcdCMD) {
    		case 1:
    			lcd_on();
    			break;
    		case 2:
    			lcd_off();
    		default:
    			break;
    		}

    		lcdCMD = 0;
    	}

    	if (readUDP) {
    		int packetSize = WiFiUDP_available();

    		if (packetSize) {
				readUDPBuffer();



				strcpy(replyBuffer, "reply");
				replyUDP();
    		}
    		readUDP = 0;
    	}

    	/*if (wifiConnected) {
			int packetSize = WiFiUDP_available();

			if (packetSize) {
				//uart_printf("packetSize: %i\r\n", packetSize);
				memset(packetBuffer, 0, sizeof(packetBuffer));
				int len = WiFiUDP_read_buffer((unsigned char*) packetBuffer, 255);
				uart_printf("Contents: %s\r\n", packetBuffer);
				if (len > 0) packetBuffer[len] = 0;

				uint32_t remoteIP = WiFiUDP_remoteIP();
				//uint8_t buffer_remoteIP[4]
				//memcpy(&buffer_remoteIP, remoteIP, 4);
				showRemoteAddress(remoteIP);

				if (strcmp(packetBuffer, CMD_STRINGS[0]) == 0) {
					uart_printf("comando aceito\r\n");
				}



				// Enviar uma resposta
				//WiFiUDP_sendBuffer((const uint8_t*) replyBuffer, sizeof(replyBuffer));
				WiFiUDP_beginPacket(remoteIP, WiFiUDP_remotePort());
				if (strcmp(packetBuffer, "teste") == 0) {
					uart_printf("reply: n1\r\n");
					strcpy(replyBuffer, "n1");
					uart_printf("replyBuffer: %s\r\n", replyBuffer);
					//WiFiUDP_write_buffer((const uint8_t*) replyBuffer, sizeof(replyBuffer)-1);
					WiFiUDP_write_buffer((const uint8_t*) replyBuffer, 2);
				}
				else {
					uart_printf("reply: n2\r\n");
					strcpy(replyBuffer, "n2");
					uart_printf("replyBuffer: %s\r\n", replyBuffer);
					//WiFiUDP_write_buffer((const uint8_t*) replyBuffer, sizeof(replyBuffer)-1);
					WiFiUDP_write_buffer((const uint8_t*) replyBuffer, 2);
				}
				WiFiUDP_endPacket();
			}
    	}*/



    	//delay(100);

    	//counter++;

    	__bis_SR_register(LPM0_bits + GIE);       // Entra no modo de baixo consumo com as interrupções habilitadas
    	__no_operation();

    }


	return 0;
}

/**
 * Leitura do UDP
 */
void readUDPBuffer() {
	memset(packetBuffer, 0, sizeof(packetBuffer));
	int len = WiFiUDP_read_buffer((unsigned char*) packetBuffer, 255);
	uart_printf("Contents: %s\r\n", packetBuffer);
	if (len > 0) packetBuffer[len] = 0;
}

/**
 * Resposta do UDP
 */
void replyUDP() {
	WiFiUDP_beginPacket(WiFiUDP_remoteIP(), WiFiUDP_remotePort());
	WiFiUDP_write_buffer((const uint8_t*) replyBuffer, strlen(replyBuffer));
	WiFiUDP_endPacket();
}

/*char* append(const char *s, char c) {
    int len = strlen(s);
    char *buf = malloc(len+2);
    strcpy(buf, s);
    buf[len] = c;
    buf[len + 1] = '\0';
    uart_printf("Str: %s\r\n", s);
    uart_printf("Add: %c\r\n", c);
    uart_printf("Buffer: %s\r\n", buf);
    return strdup(buf);
}*/

void clearLcdLine(int line) {
	lcd_setCursor(0,line);
	int i;
	for (i = 0; i < COLS; i++) {
		lcd_write(' ');
	}

}

void showIPAddress() {
	uint8_t ip_address[4];
	if (WiFi_getLocalIP((uint32_t *)ip_address)) {
		uart_printf("IP: %i.%i.%i.%i\r\n", ip_address[3], ip_address[2], ip_address[1], ip_address[0]);
		if (lcdEnabled) {

			lcd_setCursor(0,2);
			//strcpy(lcd_line, "IP: ");
			lcd_print("IP: ");

			char buffer[4];
			int i;

			for (i = 3; i >= 0; i--)
			{
				itoa(ip_address[i], buffer, 10);
				lcd_print(buffer);

				if (i > 0) {
					lcd_write('.');
				}
			}

		}
	}
}

void showRemoteAddress(uint32_t remoteIP) {
	uint8_t ip_address[4];

	ip_address[0] = (remoteIP & 0x000000ff);
	ip_address[1] = (remoteIP & 0x0000ff00) >> 8;
	ip_address[2] = (remoteIP & 0x00ff0000) >> 16;
	ip_address[3] = (remoteIP & 0xff000000) >> 24;

	uart_printf("Remote IP: %i.%i.%i.%i\r\n", ip_address[3], ip_address[2], ip_address[1], ip_address[0]);
}

void showSSID() {
	char wifi_ssid[32];
	if(WiFi_getSSID(wifi_ssid)) {
		uart_printf("SSID: %s\r\n", wifi_ssid);
		if (lcdEnabled) {
			lcd_setCursor(0,1);
			//strcpy(lcd_line, "SSID: ");
			lcd_print("SSID: ");
			lcd_print(wifi_ssid);
		}
	}
}

uint8_t connectWiFiMemory() {
	if (!wifiInit)
		return 0;

	uart_printf("Conectando pela memoria...\r\n");
	if (WiFi_fastConnect(timeoutWifi)) {
		uart_printf("Conectado!\r\n");
		return 1;
	}

	uart_printf("Não foi possível conectar pela memória!\r\n");
	return 0;
}

uint8_t connectWiFiSmartConfig() {
	if (!wifiInit)
		return 0;

	uart_printf("Inicializando o SmartConfig...\r\n");
	if (WiFi_startSmartConfig(timeoutWifi)) {
		uart_printf("Conectado pelo SmartConfig!\r\n");
		return 1;
	}

	uart_printf("Não foi possível conectar pelo SmartConfig!\r\n");
	return 0;
}

uint8_t connectWiFi() {
	if (!wifiInit) {
		return 0;
	}

	if (connectWiFiMemory())
		return 1;

	if (connectWiFiSmartConfig()) {
		return 1;
	}

	return 0;
}


void initLeds() {
	P1DIR |= BIT0;	// P1.0 output
	P1OUT &= ~BIT0;	// Disable P1.0
}

void initBuzzer() {
	BUZZER_DIR |=  BUZZER_PIN;	// Buzzer como saida
	BUZZER_OUT &= ~BUZZER_PIN;  // Desabilita o buzzer
}

void initButtons() {
	BUTTON0_DIR &= ~BUTTON0_PIN; 	// Botão 0 como entrada
	BUTTON0_OUT |=  BUTTON0_PIN; 	// Habilita o pull-up
	BUTTON0_REN |=  BUTTON0_PIN; 	// Habilita o resistor de pull-up
	BUTTON0_IES |=  BUTTON0_PIN; 	// Interrupção na transição HIGH para LOW
	BUTTON0_IE  |=  BUTTON0_PIN; 	// Interrupção habilitada
	BUTTON0_IFG &= ~BUTTON0_PIN;	// Limpa o FLAG da interrupção

	BUTTON1_DIR &= ~BUTTON1_PIN; 	// Botão 1 como entrada
	BUTTON1_REN &= ~BUTTON1_PIN; 	// Desabilita o resistor de pull-up interno
	BUTTON1_IES |=  BUTTON1_PIN; 	// Interrupção na transição HIGH para LOW
	BUTTON1_IE  |=  BUTTON1_PIN; 	// Interrupção habilitada
	BUTTON1_IFG &= ~BUTTON1_PIN;	// Limpa o FLAG da interrupção

	BUTTON2_DIR &= ~BUTTON2_PIN; 	// Botão 2 como entrada
	BUTTON2_REN &= ~BUTTON2_PIN; 	// Desabilita o resistor de pull-up interno
	BUTTON2_IES |=  BUTTON2_PIN; 	// Interrupção na transição HIGH para LOW
	BUTTON2_IE  |=  BUTTON2_PIN; 	// Interrupção habilitada
	BUTTON2_IFG &= ~BUTTON2_PIN;	// Limpa o FLAG da interrupção

	BUTTON3_DIR &= ~BUTTON3_PIN; 	// Botão 3 como entrada
	BUTTON3_REN &= ~BUTTON3_PIN; 	// Desabilita o resistor de pull-up interno
	BUTTON3_IES |=  BUTTON3_PIN; 	// Interrupção na transição HIGH para LOW
	BUTTON3_IE  |=  BUTTON3_PIN; 	// Interrupção habilitada
	BUTTON3_IFG &= ~BUTTON3_PIN;	// Limpa o FLAG da interrupção

	BUTTON4_DIR &= ~BUTTON4_PIN; 	// Botão 4 como entrada
	BUTTON4_REN &= ~BUTTON4_PIN; 	// Desabilita o resistor de pull-up interno
	BUTTON4_IES |=  BUTTON4_PIN; 	// Interrupção na transição HIGH para LOW
	BUTTON4_IE  |=  BUTTON4_PIN; 	// Interrupção habilitada
	BUTTON4_IFG &= ~BUTTON4_PIN;	// Limpa o FLAG da interrupção
}

/*
 * Configuração para o TIMER_A0 (disparado a cada 1ms)
 */
void setupTimerA0(void) {
	TA0CTL = TASSEL_2 +		// Fonte do clock: SMCLK (25MHz)
			 MC_1 +			// Modo de contagem: progressiva (crescente)
			 ID_3 +			// Fator de divisão: 8 ( 3125kHz )
			 TACLR;         // Limpa contador

	TA0CCTL0 = CCIE;        // Habilita interrupção do Timer A Bloco CCR0
	TA0CCR0 = 3125;			// Valor a ser comparado: 3125 --> 3125/3125kHz = 1ms
}

/**
 * Configuração para o TIMER_A2
 */
void setupTimerA2(void) {
	TA2CTL = TASSEL_1 +		// Fonte do clock: ACLK (32768 Hz)
			 MC_1 +			// Modo de contagem: progressiva (crescente)
			 ID_0 +			// Fator de divisão: 1 ( 32768 Hz = 0,03ms )
			 TACLR;         // Limpa contador

	TA2CCR0 = 3276;			// Valor a ser comparado: 3276 --> 3276/32768 = ~100ms
	TA2CCTL0 = CCIE;        // Habilita interrupção do Timer A Bloco CCR0

}

void setupTimers(void) {
	setupTimerA0();
	setupTimerA2();
}

/**
 * Versão do CC3000
 */
void printFirmwareWifiVersion() {
  unsigned char ver[] = {0, 0};
  WiFi_getFirmwareVersion(ver);
  uart_printf("Version CC3000: %u.%u\r\n", ver[0], ver[1]);
}


/*
 * Disparado por UART RX
 * Para sair do modo de baixo consumo em determinado caracter recebido
 */
void uart_rx_auxiliar(uint8_t c) {}

void wakeUpI2C() {
	FLAG_wakeUpI2C = 1;
}

/*
 * Interrupção UART (USCI_A1)
 */
__attribute__ ((interrupt(USCI_A1_VECTOR)))
void USCI_A1_ISR (void)
{
  switch(UCA1IV)
  {
  	  case USCI_UCRXIFG: uart_rx(); break;
  	  case USCI_UCTXIFG: break;
  }

  __bic_SR_register_on_exit(LPM4_bits);
}

/*
 * Interrupção I2C (USCI_B1)
 */
__attribute__ ((interrupt(USCI_B1_VECTOR)))
void USCI_B1_ISR (void)
{
	switch(__even_in_range(UCB1IV,12))
	{
		case  0: break;			// Vector  0: No interrupts
		case  2: break;	        // Vector  2: ALIFG
		case  4: break;         // Vector  4: NACKIFG
		case  6: break;         // Vector  6: STTIFG
		case  8: break;         // Vector  8: STPIFG
		case 10:                // Vector 10: RXIFG
			//counterRX++;
			i2c_rx();
			break;
		case 12:                // Vector 12: TXIFG
			//counterTX++;
			i2c_tx();
			break;
		default: break;
	}

	// Sair do modo de baixo consumo caso solicitado
	if (FLAG_wakeUpI2C) {
		FLAG_wakeUpI2C = 0;
		__bic_SR_register_on_exit(LPM0_bits); // Sair do modo LPM0
	}

}

/*
 * Interrupção do WatchDog
 */
__attribute__((interrupt(WDT_VECTOR)))
void WDT_ISR (void)
{
	// Copia para variaveis locais para que possam ser armazenadas em registros
	// (variaveis volateis devem ser lidas da memoria em cada acesso)
	unsigned long m = wdt_millis;
	unsigned int f = wdt_fract;

	m += sleeping ? SMILLIS_INC : MILLIS_INC;
	f += sleeping ? SFRACT_INC : FRACT_INC;
	if (f >= FRACT_MAX) {
		f -= FRACT_MAX;
		m += 1;
	}

	wdt_fract = f;
	wdt_millis = m;
	wdt_overflow_count++;

	/* Sair do modo de baixo consumo */
	__bic_SR_register_on_exit(LPM0_bits);
}


/**
 * - Interrupção para o botão 1
 * - Interrupção para o botão 2
 * - Interrupção para o botão 3
 * - Interrupção para o botão 2
 */
__attribute__((interrupt(PORT1_VECTOR)))
void PORT1_ISR (void) {
	if (P1IFG & BUTTON1_PIN) {
		uart_printf("Interrupção do botao 1!\r\n");
		if (P1IES & BUTTON1_PIN) {
			uart_printf("Botao 1 DOWN!\r\n");
		}
		else {
			uart_printf("Botao 1 UP!\r\n");
		}
		P1IFG &= ~BUTTON1_PIN;	 // Limpa o flag de interrupção
		__delay_cycles(400000); // Anti-bounce
		P1IES ^=  BUTTON1_PIN;	// Inverte a borda
	}

	if (P1IFG & BUTTON2_PIN) {
		uart_printf("Interrupção do botao 2!\r\n");
		if (P1IES & BUTTON2_PIN) {
			uart_printf("Botao 2 DOWN!\r\n");
			BUZZER_OUT |= BUZZER_PIN; // Habilita o buzzer
		}
		else {
			uart_printf("Botao 2 UP!\r\n");
			BUZZER_OUT &= ~BUZZER_PIN; // Desabilita o buzzer
		}
		P1IFG &= ~BUTTON2_PIN;	// Limpa o flag de interrupção
		__delay_cycles(400000); // Anti-bounce
		P1IES ^=  BUTTON2_PIN;	// Inverte a borda
	}

	if (P1IFG & BUTTON3_PIN) {
		uart_printf("Interrupção do botao 3!\r\n");
		if (P1IES & BUTTON3_PIN) {
			uart_printf("Botao 3 DOWN!\r\n");
		}
		else {
			uart_printf("Botao 3 UP!\r\n");
			lcdCMD = 1;
		}
		P1IFG &= ~BUTTON3_PIN;	// Limpa o flag de interrupção
		__delay_cycles(400000); // Anti-bounce
		P1IES ^=  BUTTON3_PIN;	// Inverte a borda
	}

	if (P1IFG & BUTTON4_PIN) {
		uart_printf("Interrupção do botao 4!\r\n");
		if (P1IES & BUTTON4_PIN) {
			uart_printf("Botao 4 DOWN!\r\n");
			lcdCMD = 2;
		}
		else {
			uart_printf("Botao 4 UP!\r\n");
		}
		P1IFG &= ~BUTTON4_PIN;	// Limpa o flag de interrupção
		__delay_cycles(400000); // Anti-bounce
		P1IES ^=  BUTTON4_PIN;	// Inverte a borda
	}
}


/*
 * - Interrupção necessária para o funcionamento do CC3000
 * - Interrupção para o botão 0
 */
__attribute__((interrupt(PORT2_VECTOR)))
void PORT2_ISR (void)
{
	if (P2IFG & WLAN_IRQ_PIN) {
		IntSpiGPIOHandler();
		P2IFG &= ~WLAN_IRQ_PIN;
	}

	if (P2IFG & BUTTON0_PIN) {
		uart_printf("Interrupção do botao 0!\r\n");
		P2IFG &= ~BUTTON0_PIN;
	}
	//counterSPI++;
}

/**
 * Timer para verificação de pacotes UDP (disparado a cada 1ms)
 */
__attribute__((interrupt(TIMER0_A0_VECTOR)))
void TIMER1_A0_ISR (void)
{
	if (udpInitialized && readUDP == 0) {
		readUDP = 1;
		/* Sair do modo de baixo consumo */
		//__bic_SR_register_on_exit(LPM0_bits);
	}
	/*if (udpInitialized) {
		int packetSize = WiFiUDP_available();

		if (packetSize) {
			uart_printf("Pacote recebido\r\n");
			//uart_printf("packetSize: %i\r\n", packetSize);
			memset(packetBuffer, 0, sizeof(packetBuffer));
			int len = WiFiUDP_read_buffer((unsigned char*) packetBuffer, 255);
			uart_printf("Contents: %s\r\n", packetBuffer);
			if (len > 0) packetBuffer[len] = 0;
		}
	}*/
}

/**
 * Timer para o controle dos ESCs
 */
__attribute__((interrupt(TIMER1_A0_VECTOR)))
void TIMER1_A1_ISR (void) {
	esc_timer();
}

/**
 * Timer para atualização (disparado a cada 100ms)
 */
__attribute__((interrupt(TIMER2_A0_VECTOR)))
void TIMER2_A2_ISR (void)
{
  //P1OUT ^= 0x01; // Toggle P1.0
}


