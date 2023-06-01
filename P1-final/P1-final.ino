/* 
PRACTICA 1: IMPLEMENTACION DE UN CONTROLADOR PARA UN MOTOR DE DC

*/

// DECLARACIONES //////////////////////////////////////////////////////////////////////////
#include <math.h>
// ACTIVACION DE CODIGO

#define NOMBRE_PRAC "P1-FINAL-5"
#define VERSION_SW "1.0"


// #define ACTIVA_P1
// #define DEBUG_P1A
#define ACTIVA_P1B1
#define ACTIVA_P1B2
#define ACTIVA_P1B3
#define ACTIVA_P1C
#define DEBUG_P1C
// #define ACTIVA_P1D2
#define ACTIVA_P1D3

#define ACTIVA_P1A
#define ACTIVA_P1FINAL

// Display OLED ///////////////////////////////////////////////////////////////////////////
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)

// Parametros cola de la interrupcion del encoder ///////////////////////////////////////
#define TAM_COLA_I 1024 /*num mensajes*/
#define TAM_MSG_I 1 /*num caracteres por mensaje*/

// TIEMPOS
#define BLOQUEO_TAREA_LOOPCONTR_MS 10
#define BLOQUEO_TAREA_MEDIDA_MS 100

// Configuración PWM  ////////////////////////////////////////////////////////////////////
uint32_t pwmfreq = 1000; // 1KHz
const uint8_t pwmChannel = 0;
const uint8_t pwmresolution = 8;
const int PWM_Max = pow(2,pwmresolution)-1; //

// Pines driver motor ////////////////////////////////////////////////////////////////////
const uint8_t PWM_Pin = 32; // Entrada EN 
const uint8_t PWM_f = 16; // Entrada PWM1 
const uint8_t PWM_r = 17; // Entrada PWM2 

// Voltaje maximo motor ////////////////////////////////////////////////////////////////////
float SupplyVolt = 12;

// Pines encoder ////////////////////////////////////////////////////////////////////
const uint8_t A_enc_pin = 35;
const uint8_t B_enc_pin = 34;

// Conversión a angulo y velocidad del Pololu 3072
//const float conv_rad = ; 
//const float conv_rev = ;
//const float conv_rad_grados = ; 

// Declarar funciones ////////////////////////////////////////////////////////////////////
void config_sp(); // Configuracion puerto serie
void config_oled(); // Configuracion OLED
void config_enc(); // Configuracion del encoder
void config_PWM(); // Configuracion PWM
void excita_motor(float v_motor); // Excitacion motor con PWM
//float interpola_vel_vol_lut(float x); // Interpolacion velocidad/voltios LUT

// TABLA VELOCIDAD-VOLTAJE P1D
#ifdef ACTIVA_P1D2
#define LONG_LUT 12
//Vector de tensiones
const float Vol_LUT[LONG_LUT] = {0, 1.5, 1.9, 2, 3, 4, 5, 6, 7, 8, 9, 100};
// Vector de velocidades
const float Vel_LUT[LONG_LUT] = {0, 0, 1, 1.17, 3.5, 5.33, 6.58, 7.5, 8, 8.5, 8.75, 9.16};
#endif

// Variables globales ////////////////////////////////////////////////////////////////////
int32_t ang_cnt = 0;
float pwm_volt = 0;
int32_t pwm_motor = 0;
//int32_t sign_v_ant = 0;
float v_medida = 0;    // Valor medido de angulo o velocidad -----------------
float ref_val = 0;     // Valor de referencia de angulo o velocidad
int8_t start_stop = 0; //1 -> en funcionamiento | 0 -> parado 
float K_p = 3.833; //3,833 velocidad : 1,2 ANGULO
float T_i = 1; //1 : 10 ANGULO
float T_d = 0.002; //0.002 : 0.05 ANGULO
float K_i = K_p / T_i;
float K_d = K_p * T_d;
float T_cb = 1;

bool angulo = false;
int windup_recalculo = 0;
int windup_cond = 0;
float windup_sat = 9;
int zona_muerta = 0;
float deadzone = 1.8;
float v = 0;

// Declaracion objetos  ////////////////////////////////////////////////////////////////////

xQueueHandle cola_enc; // Cola encoder
int32_t ang_enc;

/*
 RUTINAS ATENCION INTERRUPCIONES ########################################################################
*/

/* 
 Rutina de atención a interrupción ISC_enc --------------------------------------------
*/

void IRAM_ATTR ISR_enc() {
	// Lee las salidas del Encoder		
	uint8_t valor_A = digitalRead(A_enc_pin);
	uint8_t valor_B = digitalRead(B_enc_pin); 
	// Procesa los datos

  uint8_t valor_combinado = 2*valor_A + valor_B;

	// Enviar los bytes a la cola 
	if (xQueueSendFromISR( cola_enc , &valor_combinado ,NULL) != pdTRUE)
	{
	  printf("Error de escritura en la cola cola_enc \n");
	};
}

/*
 TAREAS #############################################################################
*/

/*
 Tarea task_enc #####################################################################
*/
#ifdef ACTIVA_P1A
void task_enc(void* arg) {
	// Declaracion de variables locales
  uint8_t valor_combinado;
  uint8_t valor_anterior = 0;
  uint8_t pasos_1_vuelta = 64*18.75f;
	while(1){
		// Espera a leer los datos de la cola
		if (xQueueReceive( cola_enc , &valor_combinado ,(TickType_t) portMAX_DELAY) == pdTRUE){
			// Codificar la fase del encoder
      if(valor_combinado == 3) {
        valor_combinado = 2;
      } 
      else if(valor_combinado == 2) {
        valor_combinado = 3;
      }
			// Calcular incremento/decremento y actualizar valor 
      if(valor_anterior == 0 && valor_combinado == 3) {
        ang_cnt--;
      } else if(valor_anterior == 3 && valor_combinado == 0) {
        ang_cnt++;
      } else if(valor_anterior < valor_combinado) {
        ang_cnt++;
      } else if(valor_anterior > valor_combinado) {
        ang_cnt--;
      }
      valor_anterior = valor_combinado;

			#ifdef DEBUG_P1A
				// Enviar al monitor serie
        Serial.print("ang_cnt: ");
        Serial.println(ang_cnt);

			#endif
		// } else {
		// 	printf("Error de lectura de la cola cola_enc \n");
		}
	
	}
}
#endif


/* 
Tarea de configuración de parámetros  #####################################################################
*/
void task_config(void *pvParameter) {
	char ini_char = '0';
	while(1) { 
    ini_char = Serial.read();
		// Detectar caracter enviado
    if(ini_char == 'V') {
    // Guardar valor recibido
    pwm_volt = float(Serial.parseFloat());
		// Escribir el valor recibido en la consola
    Serial.print("Voltaje motor= ");
    Serial.println(pwm_volt);
    }
    else if(ini_char == 'A') {
    // Guardar valor recibido
      ref_val = float(Serial.parseFloat());
      Serial.print("Valor de referencia= ");
      Serial.print(ref_val);
      Serial.println(" º");
      if(!angulo) {
        K_p = 1.2;
        T_i = 10;
        T_d = 0.05;
        angulo = true;
      }
    }
    else if(ini_char == 'R') {
      ref_val = float(Serial.parseFloat());
      Serial.print("Valor de referencia= ");
      Serial.print(ref_val);
      Serial.println(" rps");
      if(angulo) {
        K_p = 3.833;
        T_i = 1;
        T_d = 0.002;
        angulo = false;
      }
    }
    else if(ini_char == 'S') {
      start_stop = int(Serial.parseInt());
      if(start_stop == 1) {
        Serial.println("--START--");
      }
      else {
        Serial.println("--STOP--");
      }
    }
    else if(ini_char == 'P') {
      K_p = float(Serial.parseFloat());
      Serial.print("K_p= ");
      Serial.println(K_p);
    }
    else if(ini_char == 'I') {
      T_i = float(Serial.parseFloat());
      Serial.print("T_i= ");
      Serial.println(T_i);
      if(T_i == 0){
        K_i = 0;
      }
      else {
        K_i = K_p / T_i;
      }
    }
    else if(ini_char == 'D') {
      T_d = float(Serial.parseFloat());
      Serial.print("T_d= ");
      Serial.println(T_d);
      K_d = K_p * T_d;
    }
    else if(ini_char == 'C') {
      windup_cond = int(Serial.parseInt());
      if(windup_cond == 1) {
        Serial.println("---CONDICIONAL---");
      }
      else {
        Serial.println("--NO CONDICIONAL--");
      }
    }
    else if(ini_char == 'W') {
      windup_recalculo = int(Serial.parseInt());
      if(windup_recalculo == 1) {
        Serial.println("---RECALCULO---");
      }
      else {
        Serial.println("--NO RECALCULO--");
      }
    }
    else if(ini_char == 'L') {
      windup_sat = Serial.parseInt();
      Serial.print("windup_sat= ");
      Serial.println(windup_sat);
    }
    else if(ini_char == 'B') {
      T_cb = float(Serial.parseFloat());
      Serial.print("T_cb= ");
      Serial.println(T_cb);
    }
    else if(ini_char == 'M') {
      zona_muerta = int(Serial.parseInt());
      if(zona_muerta == 1) {
        Serial.println("---ANTI ZONA MUERTA---");
        //Ti = 15
      }
      else {
        Serial.println("--NO ANTI ZONA MUERTA--");
      }
    }

		// Activacion de la tarea cada 0.1s
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}

/* 
Tarea del lazo principal del controlador  #####################################################################
*/
float v_medida_anterior = 0;
float vi_ant = 0;
float error_ant = 0;
float sat_anterior = 0;
float resta = 0;
#ifdef ACTIVA_P1B3
void task_loopcontr(void* arg) {
  #ifdef ACTIVA_P1FINAL
    while(1) {
      if(start_stop == 1){
        float v_medida_nuevo = ang_cnt*(2*PI/1200);
        float error = 0;

        if(angulo) {
          v_medida = v_medida_nuevo; //rad
          v_medida_anterior = v_medida_nuevo;

          error = ref_val - (v_medida * (180/PI));
        }
        else {
          v_medida = (v_medida_nuevo-v_medida_anterior) / 0.01; //rad/s
          v_medida = v_medida / (2*PI); //rps

          v_medida_anterior = v_medida_nuevo;
          error = ref_val - v_medida;
        }

        float vp = K_p * error;
        float vi = 0;
        if(windup_recalculo == 1) {
        vi = vi_ant + (K_i*error - resta/T_cb)*(BLOQUEO_TAREA_LOOPCONTR_MS/1000.0);
        }
        else {
          vi = vi_ant + K_i * (BLOQUEO_TAREA_LOOPCONTR_MS/1000.0) * error;
        }
        float vd = K_d/(BLOQUEO_TAREA_LOOPCONTR_MS/1000.0) * (error - error_ant);

        v = vp + vi + vd;

        if(windup_cond == 1) {
          if(abs(v) > windup_sat) {
            v = vp + vd;
          }
        }
        else if(windup_recalculo == 1) {
          if(abs(v) > windup_sat) {
            if(v>0) {
              resta = v-windup_sat;
            }
            else {
              resta = v+windup_sat;
            }
          }
          else {
            resta = 0;
          }
        }

        vi_ant = vi;
        error_ant = error;

        if(zona_muerta == 1) {
          if(v>0) {
            if(v > 0.3) {
              v = v+deadzone;
            }
          }
          else {
            if(v<-0.3){
              v = v-deadzone;
            }
          }
        }
        excita_motor(v);
      }
      else {
        excita_motor(pwm_volt);
        ang_cnt = 0;
        pwm_motor = 0;
        v_medida = 0;
        v_medida_anterior = 0;
        vi_ant = 0;
        error_ant = 0;
        resta = 0;
      }
	  	vTaskDelay(BLOQUEO_TAREA_LOOPCONTR_MS / portTICK_PERIOD_MS);
    }
  #endif

  // Codigo anterior
  #ifdef ACTIVA_P1
	  while(1) {
	  	// Excitacion del motor con PWM
	  	if(start_stop == 1){
        excita_motor(pwm_volt);
        float v_medida_nuevo = ang_cnt*(2*PI/1200);
        float error = ref_val - v_medida;

        #ifdef ACTIVA_P1C_MED_ANG // Medida de angulo

	  	  #else // Medida de velocidad

          v_medida = (v_medida_nuevo-v_medida_anterior) / 0.01; //rad/s
          v_medida = v_medida / (2*PI); //rps

          v_medida_anterior = v_medida_nuevo;
	  	  #endif

        #ifdef ACTIVA_P1D2

          pwm_volt = interpola_vel_vol_lut(ref_val);

        #endif

        #ifdef ACTIVA_P1D3

        if(error > 0) {
          pwm_volt = pwm_volt + K_p;
        }
        else if (error < 0) {
          pwm_volt = pwm_volt - K_p;
        }
        else {
          pwm_volt = pwm_volt;
        }

        #endif
      }
      else {
        excita_motor(0);
        ang_cnt = 0;
        pwm_motor = 0;
        v_medida = 0;
        v_medida_anterior = 0;
      }
	  	// Activacion de la tarea cada 0.01s
	  	vTaskDelay(BLOQUEO_TAREA_LOOPCONTR_MS / portTICK_PERIOD_MS);
	  }
  #endif
}
#endif

/* 
Tarea del lazo principal del controlador  #####################################################################
*/
#ifdef DEBUG_P1C
void task_medidas(void* arg) 
{

	while(1) { 
		if(start_stop == 1) {
      // Mostrar medidas de angulo y velocidad del motor
        if(angulo) {
          float v_medida_rad = v_medida * (180/PI);
          Serial.print("Med:");
          Serial.print(v_medida_rad);
          Serial.print(",Ref:");
          Serial.println(ref_val);
		    } 
        else { // Medida de velocidad
          Serial.print("Med:");
          Serial.print(v_medida);

          Serial.print(",Ref:");
          Serial.println(ref_val);
          Serial.print(",v:");
          Serial.println(v);
        }
    }
		// Activacion de la tarea cada 1s
		vTaskDelay(BLOQUEO_TAREA_MEDIDA_MS / portTICK_PERIOD_MS);

	}
}
#endif

/*
SET UP -----------------------------------------------------------------------------------
*/
void setup() {
	// Configuracion puerto serie
	config_sp();
	
	// Configuracion OLED
	config_oled();

	// Configuracion PWM
  config_PWM();

	// Crear cola_enc
	cola_enc = xQueueCreate(TAM_COLA_I, TAM_MSG_I);
	if(cola_enc == NULL) {
	 	Serial.println("Error en creacion de cola_enc");
	 	exit(-1);
	};

	// Crear la tarea task_enc
	if(xTaskCreate( task_enc, "task_enc", 2048, NULL, 2, NULL) != pdPASS) {
	 	Serial.println("Error en creacion tarea task_enc");
	 	exit(-1);
	}

	// Crear la tarea task_config
  if(xTaskCreate( task_config, "task_config", 2048, NULL, 1, NULL) != pdPASS) {
	 	Serial.println("Error en creacion tarea task_config");
	 	exit(-1);
	}

	// Crear la tarea task_loopcontr
  if(xTaskCreate( task_loopcontr, "task_loopcontr", 2048, NULL, 2, NULL) != pdPASS) {
	 	Serial.println("Error en creacion tarea task_loopcontr");
	 	exit(-1);
	}

	#ifdef DEBUG_P1C
		// Crear la tarea task_medidas
    if(xTaskCreate( task_medidas, "task_medidas", 2048, NULL, 1, NULL) != pdPASS) {
	 	Serial.println("Error en creacion tarea task_medidas");
	 	exit(-1);
	}

	#endif

	// Configuracion del encoder
  config_enc();
}

/*
LOOP ---- NO USAR ------------------------------------------------------------------- 
*/
void loop() {}

// FUNCIONES ////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
// Funcion configuracion del encoder
////////////////////////////////////////////////////////////////////////////////////
#ifdef ACTIVA_P1A
void config_enc(){
	// Configuracion de pines del encoder
    pinMode(A_enc_pin, INPUT);
    pinMode(B_enc_pin, INPUT);
	// Configuracion interrupciones
    attachInterrupt(A_enc_pin, ISR_enc, CHANGE);
    attachInterrupt(B_enc_pin, ISR_enc, CHANGE);
} 
#endif
////////////////////////////////////////////////////////////////////////////////////
// Funcion configuracion del PWM
////////////////////////////////////////////////////////////////////////////////////
#ifdef ACTIVA_P1B2
void config_PWM(){
	// Configuracion de pines de control PWM
  pinMode(PWM_f, OUTPUT);
  pinMode(PWM_r, OUTPUT);
	// Configuracion LED PWM 
  ledcSetup(pwmChannel,pwmfreq,pwmresolution);
	// Asignar el controlador PWM al GPIO
  ledcAttachPin(PWM_Pin, 0);

}  
#endif

////////////////////////////////////////////////////////////////////////////////////
// Funcion excitacion del motor con PWM
////////////////////////////////////////////////////////////////////////////////////
#ifdef ACTIVA_P1B3
void excita_motor(float v_motor){
	// Sentido de giro del motor
  float v_motor_ant = 0;
  if((v_motor_ant < 0 && v_motor > 0) || (v_motor_ant > 0 && v_motor < 0)) {
    digitalWrite(PWM_f, LOW);
    digitalWrite(PWM_r, LOW);
    v_motor_ant = v_motor;
    Serial.println("Cambiando sentido de giro");
  }

  if(v_motor > 0) {
    digitalWrite(PWM_f, HIGH);
    digitalWrite(PWM_r, LOW);
  }
  else {
    digitalWrite(PWM_f, LOW);
    digitalWrite(PWM_r, HIGH);
  }
	// Calcula y limita el valor de configuración del PWM
  if(abs(v_motor) > 12) {
    v_motor = 12;
  }
	// El valor de excitación debe estar entro 0 y PWM_Max
  pwm_motor = (PWM_Max * abs(v_motor))/12;
	
	// Excitacion del motor con PWM
	ledcWrite(0, pwm_motor);
}  
#endif

////////////////////////////////////////////////////////////////////////////////////
// Funcion configuracion del puerto serie
////////////////////////////////////////////////////////////////////////////////////
void config_sp(){
	Serial.begin(115200);
	Serial.println("  ");
	Serial.println("--------------------------------------------");
	Serial.println("PRACTICA CONTROLADOR MOTOR " NOMBRE_PRAC);
	Serial.println("--------------------------------------------");
}  

////////////////////////////////////////////////////////////////////////////////////
// Funcion configuracion del OLED
////////////////////////////////////////////////////////////////////////////////////
void config_oled(){
	Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
	display.begin(SSD1306_SWITCHCAPVCC, 0x3C) ;
	display.clearDisplay();
	display.setTextColor(WHITE);        // 
	display.setCursor(0,0);             // Start at top-left corner
	display.println(F("CONTR. MOTOR " NOMBRE_PRAC));
	display.display();
	delay(1000);
	display.setTextColor(BLACK,WHITE);        // 
	display.setCursor(0,20);             // Start at top-left corner
	display.println(F(" SW v." VERSION_SW));
	display.display();
	delay(1000);
}  


////////////////////////////////////////////////////////////////////////////////////
// Funcion de interpolacion LUT de Velocidad-Voltaje
////////////////////////////////////////////////////////////////////////////////////
#ifdef ACTIVA_P1D2
float interpola_vel_vol_lut(float x) {
	// Buscar el valor superior más pequeño del array
	int8_t i = 0;
	if ( x >= Vel_LUT[LONG_LUT - 2] ) {i = LONG_LUT - 2;}
	else {while ( x > Vel_LUT[i+1] ) i++;}

	// Guardar valor superior e inferior
	float xL = Vel_LUT[i];
	float yL = Vol_LUT[i];
	float xR = Vel_LUT[i+1];
	float yR = Vol_LUT[i+1];

	// Interpolar
	float dydx = ( yR - yL ) / ( xR - xL );

	return yL + dydx * ( x - xL );
}
#endif