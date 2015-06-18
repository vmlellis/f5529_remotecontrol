/*
 * main.c
 *
 *  Construido com a IDE CCSv6 e compilador GNU v4.9.1 (Red Hat)
 */


#include <msp430f5529.h>
#include <math.h>
#include "setup.h"
#include "uart/uart_tx.h"
#include "i2c/lcd/lcd.h"
#include "i2c/lcd/lcd_blue.h"
#include "spi/cc3000/WiFi.h"
#include "spi/cc3000/WiFiUDP.h"
#include "libraries/utils.h"
#include "libraries/esc.h"
#include "libraries/buzzer_sounds.h"
#include "i2c/ms5611/ms5611.h"
#include "i2c/mpu6050/mpu6050.h"
#include "i2c/hmc5883l/hmc5883l.h"
#include "libraries/madgwick.h"

#define UDP_PORT 2390

#define RESOLUTION_16BIT 32768
#define ACCEL_AFS 2
#define GYRO_DPS 250

#define GyroMeasError PI * (40.0f / 180.0f)    // gyroscope measurement error in rads/s (shown as 3 deg/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
#define GyroBeta sqrt(3.0f / 4.0f) * GyroMeasError   // compute beta


/*volatile int counter = 0;
volatile int counterRX = 0;
volatile int counterTX = 0;
volatile int counterSPI = 0;*/
volatile int counter_timer = 0;



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

// Parametros do magnometro (HMC5883L)
uint8_t magEnabled = 0;			// Informa que o magnometro estah habilitado
int mx = 0, my = 0, mz = 0;		// Dados do magnometro

// Parametros do barometro (MS5611)
uint8_t barEnabled = 0;
float altitude = 0.0;

// Parametros do accel/gyro (MPU6050)
uint8_t mpuEnabled = 0;
int16_t gx = 0, gy = 0, gz = 0;	// Dados do giroscopio
int16_t ax = 0, ay = 0, az = 0; // Dados do acelerometro
float axNormalized, ayNormalized, azNormalized; // Aceleração normalizada
float gxNormalized, gyNormalized, gzNormalized; // Giroscopio normalizado

// Madgwick Filter
madgwick_params mParams;
uint32_t lastUpdate = 0; // used to calculate integration interval
uint32_t now = 0;        // used to calculate integration interval

// Dados para o sistema de orientacao
float pitch, yaw, roll;

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
char replyBuffer[255];
/*char replyBuffer[] = "acknowledged";
char replyBuffer2[] = "teste ok";*/
void showIPAddress(void);
void showRemoteAddress(uint32_t);
void showSSID(void);
uint8_t connectWiFi(void);
uint8_t connectWiFiMemory(void);
uint8_t connectWiFiSmartConfig(void);
void printFirmwareWifiVersion(void);
volatile int packetSize;
void stepsAfterConnectWiFi(void);

uint32_t pemitted_remoteIP = 0;
//const char *CMD_START_STRINGS[] = { "WIFI_", "BUZZER_", "ESC1_", "ESC2_", "ESC3_", "ESC4_" };
const char *CMD_WIFI[] = { "WIFI_CONNECT", "WIFI_DISCONNECT" };


// Prototipos
//char* append(const char *s, char c);

uint8_t endsWith(char *str, char *suffix);
uint8_t startsWith(char *str, char *pre);
void printIndex();

//char current_line[100] = "";
char lcd_line[21] = "";


volatile uint8_t readUDP = 0;
volatile uint8_t buttonCMD = 0;
void readUDPBuffer(void);
void replyUDP(void);

// Buzzer - Music Control
uint8_t buzzer_sing = 0;
int buzzer_nota_index = 0;
volatile int buzzer_counter_ms = 0; // Utilizado para contar os ms




/**
 * Caracteres especiais para o LCD
 * - http://www.arduino.cc/en/Reference/LiquidCrystalCreateChar
 * - https://www.hackmeister.dk/2010/08/custom-lcd-characters-with-arduino/
 * - http://forum.arduino.cc/index.php?topic=74666.0
 * - https://www.youtube.com/watch?v=jaD0dJtovwc
 * - http://learn.trossenrobotics.com/28-robotgeek-getting-started-guides/59-lcd-special-characters.html
 */
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
    uart_printf("I2C inicializado!\r\n");

    setupTimers();
    uart_printf("Timers inicializados!\r\n");

    magEnabled = 0;
    barEnabled = 0;
    mpuEnabled = 0;
    lcdEnabled = 0;

    buttonCMD = 0;

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

    // Inicializar os ESCs
    esc_init();


    // Inicialização do MPU6050
    if (mpu6050_detect()) {
    	mpu6050_config();
		mpuEnabled = 1;
    }


    // Inicialização do HMC5883L
    if (hmc5883l_detect()) {
    	hmc5883l_config();
    	hmc5883l_calibrate();
		magEnabled = 1;
    }

    // Inicialização do MS5611
    if (ms5611_detect()) {
    	ms5611_config(MS5611_ULTRA_HIGH_RES);
		barEnabled = 1;
    }

    if (mpuEnabled && magEnabled) {
    	madgwick_init(&mParams, 0.0f, GyroBeta);
    	uart_printf("q -> %.3f:%.3f:%.3f:%.3f\r\n",
    			mParams.quaternion[0], mParams.quaternion[1], mParams.quaternion[2], mParams.quaternion[3]);

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
		stepsAfterConnectWiFi();
		/*if (wifiConnected) {
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
		}*/
    }



    while (1) {

    	if (buttonCMD > 0) {
    		switch(buttonCMD) {
    		case 1:
    			lcd_on();
    			break;
    		case 2:
    			lcd_off();
    			break;
    		case 3:
    			//buzzer_sing = 1;
    			uart_printf("SmartConfig!\r\n");
    			lcd_setCursor(0,1);
    			strcpy(lcd_line, "WiFi: SmartConfig...");
    			wifiConnected = connectWiFiSmartConfig();
    			stepsAfterConnectWiFi();
    			break;
    		default:
    			break;
    		}

    		buttonCMD = 0;
    	}

    	if (buzzer_sing) {
    		if (playMarioSong(&buzzer_nota_index)) {
    			buzzer_sing = 0;
    			buzzer_nota_index = 0;
    			resetPausa();
    		}
    	}

    	if (readSensors) {
    		/**
    		 * Referencia IMU:
    		 * - https://github.com/kriswiner/MPU6050HMC5883AHRS/blob/master/MPU6050HMC5883AHRS.ino
    		 * - https://github.com/kriswiner/MPU-6050/wiki/Affordable-9-DoF-Sensor-Fusion
    		 */
    		if (mpuEnabled) {
				mpu6050_getAcceleration(&ax, &ay, &az);

				// Calculado o valor da aceleracao em g's
				float resolution = (float) ACCEL_AFS / (float) RESOLUTION_16BIT;
				axNormalized = ax * resolution;
				ayNormalized = ay * resolution;
				azNormalized = az * resolution;

				mpu6050_getRotation(&gx,&gy,&gz);

				// Calcular o valor atual de graus por segundo
				resolution = (float) GYRO_DPS / (float) RESOLUTION_16BIT;
				gxNormalized = gx * resolution;
				gyNormalized = gy * resolution;
				gzNormalized = gz * resolution;


    		}

    		if (magEnabled) {
    			hmc5883l_read_scalled_data(&mx,&my,&mz);
    		}


    		if (barEnabled)
    			ms5611_readAltitude(&altitude);

    		if (mpuEnabled && magEnabled) {
				now = micros();
				mParams.samplePeriod = ((now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
				lastUpdate = now;

				madgwick_update(&mParams, ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  mx,  my,  mz);

				// Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
				// In this coordinate system, the positive z-axis is down toward Earth.
				// Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
				// Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
				// Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
				// These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
				// Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
				// applied in the correct order which for this configuration is yaw, pitch, and then roll.
				// For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
				yaw   = atan2(2.0f * (mParams.quaternion[1] * mParams.quaternion[2] + mParams.quaternion[0] * mParams.quaternion[3]),
						mParams.quaternion[0] * mParams.quaternion[0] + mParams.quaternion[1] * mParams.quaternion[1] - mParams.quaternion[2] * mParams.quaternion[2] - mParams.quaternion[3] * mParams.quaternion[3]);
				pitch = -asin(2.0f * (mParams.quaternion[1] * mParams.quaternion[3] - mParams.quaternion[0] * mParams.quaternion[2]));
				roll  = atan2(2.0f * (mParams.quaternion[0] * mParams.quaternion[1] + mParams.quaternion[2] * mParams.quaternion[3]),
						mParams.quaternion[0] * mParams.quaternion[0] - mParams.quaternion[1] * mParams.quaternion[1] - mParams.quaternion[2] * mParams.quaternion[2] + mParams.quaternion[3] * mParams.quaternion[3]);

				pitch *= 180.0f / PI;
				yaw   *= 180.0f / PI;
				roll  *= 180.0f / PI;

    		}

    		readSensors = 0;
    	}

    	if (readUDP) {
    		int packetSize = WiFiUDP_available();

    		if (packetSize) {
				readUDPBuffer();

				uint8_t cmd_error = 1;
				uint32_t remoteIP = WiFiUDP_remoteIP();

				/**
				 * Wifi Commands
				 */
				if (startsWith(packetBuffer, "WIFI_")) {
					if (strcmp(packetBuffer, CMD_WIFI[0]) == 0) {
						pemitted_remoteIP = remoteIP;
						strcpy(replyBuffer, "{status: 1, cmd: \"WIFI_CONNECT\"}");
						cmd_error = 0;
					}

					if (strcmp(packetBuffer, CMD_WIFI[1]) == 0) {
						pemitted_remoteIP = 0;
						strcpy(replyBuffer, "{status: 1, cmd: \"WIFI_DISCONNECT\"}");
						cmd_error = 0;
					}
				}

				/**
				 * Buzzer Commands
				 */
				if (startsWith(packetBuffer, "BUZZER_")) {
					if (pemitted_remoteIP == remoteIP) {
						if (strcmp(packetBuffer, "BUZZER_ON") == 0) {
							BUZZER_OUT |= BUZZER_PIN; // Habilita o buzzer
							strcpy(replyBuffer, "{status: 1, cmd: \"BUZZER_ON\"}");
							cmd_error = 0;
						}

						if (strcmp(packetBuffer, "BUZZER_OFF") == 0) {
							BUZZER_OUT &= ~BUZZER_PIN;  // Desabilita o buzzer
							strcpy(replyBuffer, "{status: 1, cmd: \"BUZZER_OFF\"}");
							cmd_error = 0;
						}

						if (strcmp(packetBuffer, "BUZZER_PLAY_MARIO") == 0) {
							buzzer_sing = 1;
							buzzer_nota_index = 0;
							resetPausa();
							strcpy(replyBuffer, "{status: 1, cmd: \"BUZZER_PLAY_MARIO\"}");
							cmd_error = 0;
						}

						if (strcmp(packetBuffer, "BUZZER_STOP_MARIO") == 0) {
							buzzer_sing = 0;
							buzzer_nota_index = 0;
							resetPausa();
							strcpy(replyBuffer, "{status: 1, cmd: \"BUZZER_STOP_MARIO\"}");
							cmd_error = 0;
						}
					}
					else {
						cmd_error = 2;
					}
				}

				/**
				 * ESC1 Commands
				 */
				if (startsWith(packetBuffer, "ESC1_")) {
					if (pemitted_remoteIP == remoteIP) {
						if (strcmp(packetBuffer, "ESC1_CONNECT") == 0) {
							esc_connect(1);
							strcpy(replyBuffer, "{status: 1, cmd: \"ESC1_CONNECT\"}");
							cmd_error = 0;
						}

						if (strcmp(packetBuffer, "ESC1_DISCONNECT") == 0) {
							esc_disconnect(1);
							strcpy(replyBuffer, "{status: 1, cmd: \"ESC1_DISCONNECT\"}");
							cmd_error = 0;
						}

						if (strcmp(packetBuffer, "ESC1_ENABLE_CONFIG") == 0) {
							esc_enableConfigMotor(1);
							strcpy(replyBuffer, "{status: 1, cmd: \"ESC1_ENABLE_CONFIG\"}");
							cmd_error = 0;
						}

						if (strcmp(packetBuffer, "ESC1_ACTIVE_CONFIG") == 0) {
							esc_activeConfigMotor(1);
							strcpy(replyBuffer, "{status: 1, cmd: \"ESC1_ACTIVE_CONFIG\"}");
							cmd_error = 0;
						}
					}
					else {
						cmd_error = 2;
					}
				}

				/**
				 * ESC2 Commands
				 */
				if (startsWith(packetBuffer, "ESC2_")) {
					if (pemitted_remoteIP == remoteIP) {
						if (strcmp(packetBuffer, "ESC2_CONNECT") == 0) {
							esc_connect(2);
							strcpy(replyBuffer, "{status: 1, cmd: \"ESC2_CONNECT\"}");
							cmd_error = 0;
						}

						if (strcmp(packetBuffer, "ESC2_DISCONNECT") == 0) {
							esc_disconnect(2);
							strcpy(replyBuffer, "{status: 1, cmd: \"ESC2_DISCONNECT\"}");
							cmd_error = 0;
						}

						if (strcmp(packetBuffer, "ESC2_ENABLE_CONFIG") == 0) {
							esc_enableConfigMotor(2);
							strcpy(replyBuffer, "{status: 1, cmd: \"ESC2_ENABLE_CONFIG\"}");
							cmd_error = 0;
						}

						if (strcmp(packetBuffer, "ESC2_ACTIVE_CONFIG") == 0) {
							esc_activeConfigMotor(2);
							strcpy(replyBuffer, "{status: 1, cmd: \"ESC2_ACTIVE_CONFIG\"}");
							cmd_error = 0;
						}
					}
					else {
						cmd_error = 2;
					}
				}

				/**
				 * ESC3 Commands
				 */
				if (startsWith(packetBuffer, "ESC3_")) {
					if (pemitted_remoteIP == remoteIP) {
						if (strcmp(packetBuffer, "ESC3_CONNECT") == 0) {
							esc_connect(3);
							strcpy(replyBuffer, "{status: 1, cmd: \"ESC3_CONNECT\"}");
							cmd_error = 0;
						}

						if (strcmp(packetBuffer, "ESC3_DISCONNECT") == 0) {
							esc_disconnect(3);
							strcpy(replyBuffer, "{status: 1, cmd: \"ESC3_DISCONNECT\"}");
							cmd_error = 0;
						}

						if (strcmp(packetBuffer, "ESC3_ENABLE_CONFIG") == 0) {
							esc_enableConfigMotor(3);
							strcpy(replyBuffer, "{status: 1, cmd: \"ESC3_ENABLE_CONFIG\"}");
							cmd_error = 0;
						}

						if (strcmp(packetBuffer, "ESC3_ACTIVE_CONFIG") == 0) {
							esc_activeConfigMotor(3);
							strcpy(replyBuffer, "{status: 1, cmd: \"ESC3_ACTIVE_CONFIG\"}");
							cmd_error = 0;
						}
					}
					else {
						cmd_error = 2;
					}
				}

				/**
				 * ESC4 Commands
				 */
				if (startsWith(packetBuffer, "ESC4_")) {
					if (pemitted_remoteIP == remoteIP) {
						if (strcmp(packetBuffer, "ESC4_CONNECT") == 0) {
							esc_connect(4);
							strcpy(replyBuffer, "{status: 1, cmd: \"ESC3_CONNECT\"}");
							cmd_error = 0;
						}

						if (strcmp(packetBuffer, "ESC4_DISCONNECT") == 0) {
							esc_disconnect(4);
							strcpy(replyBuffer, "{status: 1, cmd: \"ESC4_DISCONNECT\"}");
							cmd_error = 0;
						}

						if (strcmp(packetBuffer, "ESC4_ENABLE_CONFIG") == 0) {
							esc_enableConfigMotor(4);
							strcpy(replyBuffer, "{status: 1, cmd: \"ESC4_ENABLE_CONFIG\"}");
							cmd_error = 0;
						}

						if (strcmp(packetBuffer, "ESC4_ACTIVE_CONFIG") == 0) {
							esc_activeConfigMotor(4);
							strcpy(replyBuffer, "{status: 1, cmd: \"ESC4_ACTIVE_CONFIG\"}");
							cmd_error = 0;
						}
					}
					else {
						cmd_error = 2;
					}
				}

				/**
				 * ESC Commands
				 */
				if (startsWith(packetBuffer, "ESC_")) {
					if (pemitted_remoteIP == remoteIP) {
						if (strcmp(packetBuffer, "ESC_CONNECT_ALL") == 0) {
							esc_connectAll();
							strcpy(replyBuffer, "{status: 1, cmd: \"ESC_CONNECT_ALL\"}");
							cmd_error = 0;
						}

						if (strcmp(packetBuffer, "ESC_DISCONNECT_ALL") == 0) {
							esc_disconnectAll();
							strcpy(replyBuffer, "{status: 1, cmd: \"ESC_DISCONNECT_ALL\"}");
							cmd_error = 0;
						}

					}
					else {
						cmd_error = 2;
					}
				}

				/**
				 * Read Sensors
				 */
				if (startsWith(packetBuffer, "SENSOR_")) {
					if (pemitted_remoteIP == remoteIP) {
						if (strcmp(packetBuffer, "SENSOR_ACCEL") == 0) {
							strcpy(replyBuffer, "{status: 1, cmd: \"SENSOR_ACCEL\", ax: ");

							char buffer[8];

							itoa((int) (axNormalized * 1000), buffer, 10);
							strcat(replyBuffer, buffer);

							strcat(replyBuffer, ", ay: ");
							itoa((int) (ayNormalized * 1000), buffer, 10);
							strcat(replyBuffer, buffer);

							strcat(replyBuffer, ", az: ");
							itoa((int) (azNormalized * 1000), buffer, 10);
							strcat(replyBuffer, buffer);

							strcat(replyBuffer, " }");

							cmd_error = 0;
						}

						if (strcmp(packetBuffer, "SENSOR_GYRO") == 0) {
							strcpy(replyBuffer, "{status: 1, cmd: \"SENSOR_GYRO\", gx: ");

							char buffer[8];

							itoa((int) (gxNormalized * 1000), buffer, 10);
							strcat(replyBuffer, buffer);

							strcat(replyBuffer, ", gy: ");
							itoa((int) (gyNormalized * 1000), buffer, 10);
							strcat(replyBuffer, buffer);

							strcat(replyBuffer, ", gz: ");
							itoa((int) (gzNormalized * 1000), buffer, 10);
							strcat(replyBuffer, buffer);

							strcat(replyBuffer, " }");

							//uart_printf("%i:%i:%i\r\n", gx, gy, gz);

							cmd_error = 0;
						}

						if (strcmp(packetBuffer, "SENSOR_MAG") == 0) {
							strcpy(replyBuffer, "{status: 1, cmd: \"SENSOR_MAG\", mx: ");

							char buffer[8];

							itoa(mx, buffer, 10);
							strcat(replyBuffer, buffer);

							strcat(replyBuffer, ", my: ");
							itoa(my, buffer, 10);
							strcat(replyBuffer, buffer);

							strcat(replyBuffer, ", mz: ");
							itoa(mz, buffer, 10);
							strcat(replyBuffer, buffer);

							strcat(replyBuffer, " }");

							//uart_printf("%i:%i:%i\r\n", mx, my, mz);

							cmd_error = 0;
						}

						if (strcmp(packetBuffer, "SENSOR_QUATERNION") == 0) {
							strcpy(replyBuffer, "{status: 1, cmd: \"SENSOR_QUATERNION\", q0: ");

							char buffer[8];

							itoa((int) (mParams.quaternion[0] * 1000), buffer, 10);
							strcat(replyBuffer, buffer);

							strcat(replyBuffer, ", qx: ");
							itoa((int) (mParams.quaternion[1] * 1000), buffer, 10);
							strcat(replyBuffer, buffer);

							strcat(replyBuffer, ", qy: ");
							itoa((int) (mParams.quaternion[2] * 1000), buffer, 10);
							strcat(replyBuffer, buffer);

							strcat(replyBuffer, ", qz: ");
							itoa((int) (mParams.quaternion[3] * 1000), buffer, 10);
							strcat(replyBuffer, buffer);

							strcat(replyBuffer, " }");

							uart_printf("q -> %.3f:%.3f:%.3f:%.3f\r\n",
							    			mParams.quaternion[0], mParams.quaternion[1], mParams.quaternion[2], mParams.quaternion[3]);

							cmd_error = 0;
						}

						if (strcmp(packetBuffer, "SENSOR_ROT") == 0) {
							strcpy(replyBuffer, "{status: 1, cmd: \"SENSOR_ROT\", y: ");

							char buffer[8];

							itoa((int) (yaw * 1000), buffer, 10);
							strcat(replyBuffer, buffer);

							strcat(replyBuffer, ", p: ");
							itoa((int) (pitch * 1000), buffer, 10);
							strcat(replyBuffer, buffer);

							strcat(replyBuffer, ", r: ");
							itoa((int) (roll * 1000), buffer, 10);
							strcat(replyBuffer, buffer);

							strcat(replyBuffer, " }");


							cmd_error = 0;
						}

					}
					else {
						cmd_error = 2;
					}
				}




				if (cmd_error == 1)
					strcpy(replyBuffer, "{status: 0, msg: \"NOT FOUND\"}");

				if (cmd_error == 2)
					strcpy(replyBuffer, "{status: 0, msg: \"NOT AUTHORIZED\"}");

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

void stepsAfterConnectWiFi(void) {
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
			buttonCMD = 3;
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
			buttonCMD = 1;
		}
		P1IFG &= ~BUTTON3_PIN;	// Limpa o flag de interrupção
		__delay_cycles(400000); // Anti-bounce
		P1IES ^=  BUTTON3_PIN;	// Inverte a borda
	}

	if (P1IFG & BUTTON4_PIN) {
		uart_printf("Interrupção do botao 4!\r\n");
		if (P1IES & BUTTON4_PIN) {
			uart_printf("Botao 4 DOWN!\r\n");
			buttonCMD = 2;
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
	buzzer_counter_ms++;
}

/**
 * Timer para o controle dos ESCs
 */
__attribute__((interrupt(TIMER1_A0_VECTOR)))
void TIMER1_A1_ISR (void) {
	esc_timer();
	counter_timer++;
}

/**
 * Timer para atualização (disparado a cada 100ms)
 */
__attribute__((interrupt(TIMER2_A0_VECTOR)))
void TIMER2_A2_ISR (void)
{
	readSensors = 1;
  //P1OUT ^= 0x01; // Toggle P1.0
}


