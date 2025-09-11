/* ===== EXPERIMENTO CAIDA LIBRE ===== */
// Codigo utilizado en el experimento de caida libre diseñado para
// el curso Laboratorio 1: Mecanica I del segundo (2°) semestre de la
// carrera de Ciencias Fisicas - FCFM - Universidad de Concepcion.

/* Librerias */

//#include "Adafruit_VL53L0X.h" //Libreria para sensor ToF

#include <Arduino.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/* ============================= */
void TimeDisplay();

const int BAUDRATE = 9600;

LiquidCrystal_I2C lcd(0x27, 20, 4);

double t_inicial = 0;
double t1, t2, t3, t4 = 0;
bool start = 1;


const int laserPins[] = {3, 5, 7, 9};           // Pines conectados a los láseres
const int photoResPins[] = {4, 6, 8, 10};   // Pines conectados a las fotoresistencias
int button = 2;                       // Pin de reset de la barrera

int numPairs = sizeof(laserPins)/sizeof(laserPins[0]);  // Numero de modulos

bool photoResStates[sizeof(photoResPins)/sizeof(photoResPins[0])]; // Estados actuales
bool photoResPrevStates[sizeof(photoResPins)/sizeof(photoResPins[0])]; // Estados previos


void setup() {
  Serial.begin(BAUDRATE);
  // ====== LCD ========//
  lcd.init();
  lcd.backlight();
// =================//

  t_inicial = millis();
  pinMode(button, INPUT_PULLUP);

  //================= Inicializar pines ===============//
  for (int i = 0; i < numPairs; ++i) {
    pinMode(laserPins[i], OUTPUT);
    digitalWrite(laserPins[i], HIGH);

    pinMode(photoResPins[i], INPUT_PULLUP);

    photoResPrevStates[i] = digitalRead(photoResPins[i]);
  }
  //===========================================//
}

void loop() {

  TimeDisplay();

  //============ Restart button ============//
  //  For debugging uncomment the Serial.println line
  bool pressed = digitalRead(button); // Standby state = 0
  //Serial.println(pressed); 

  //=========== Read photoresistors ============//
  // For debugging uncomment the Serial.print lines:
  for (int i = 0; i < numPairs; ++i) {
    photoResStates[i] = digitalRead(photoResPins[i]);
    //Serial.print("Sensor "); Serial.print(i); Serial.print(": "); Serial.println(photoResStates[i]);
  }

  //=========== Reset ============//
  if (pressed) {
    t1 = 0;
    t2 = 0;
    t3 = 0;
    t4 = 0;

    t_inicial = (double)millis();
    for (int i = 0; i < numPairs; ++i) {
      digitalWrite(laserPins[i], HIGH);
      photoResPrevStates[i] = photoResStates[i]; //Reinicia estado previo.
    }
  
    lcd.clear();
    TimeDisplay();
  }

  //=========== Measure time ============//
  // En los comentarios podemos ver la implemetación de a función BarrierTrigered.

    // Para B1
    BarrierTrigered(0, t1);
    //if (photoResPrevStates[0] == LOW && photoResStates[0] == HIGH && t1 == 0) {
    //  t1 = ((double)millis() - t_inicial) / 1000.0;  // Tiempo transcurrido tras pasar primera barrera
    //  digitalWrite(laserPins[0], LOW);
    // }

    // Para B2
    BarrierTrigered(1, t2);
    //if (photoResPrevStates[1] == LOW && photoResStates[1] == HIGH && t2 == 0) {
    //  t2 = ((double)millis() - t_inicial) / 1000.0;  // Tiempo transcurrido tras pasar segunda barrera
    //  digitalWrite(laserPins[1], LOW);
    // }

    // Para B3
    BarrierTrigered(2, t3);
    //if (photoResPrevStates[2] == LOW && photoResStates[2] == HIGH && t3 == 0) {
    //  t3 = ((double)millis() - t_inicial) / 1000.0;  // Tiempo transcurrido tras pasar tercera barrera
    //  digitalWrite(laserPins[2], LOW);
    // }

    // Para B4
    BarrierTrigered(3, t4); 
   // if (photoResPrevStates[3] == LOW && photoResStates[3] == HIGH && t4 == 0) {
   //   t4 = ((double)millis() - t_inicial) / 1000.0;  // Tiempo transcurrido tras pasar cuarta barrera
   //   digitalWrite(laserPins[3], LOW);
   // }
  
  // ========== Actualizar estados previos. ========== //
  for (int i = 0; i < numPairs; ++i) {
    photoResPrevStates[i] = photoResStates[i];
  }
}

//============ Funciones (Try objects when you can.) ===========//

void TimeDisplay() {

  lcd.setCursor(0, 0);
  lcd.print("B1:");
  lcd.print(t1);
  lcd.print("s");

  lcd.setCursor(0, 1);
  lcd.print("B2:");
  lcd.print(t2);
  lcd.print("s");

  lcd.setCursor(0, 2);
  lcd.print("B3:");
  lcd.print(t3);
  lcd.print("s");

  lcd.setCursor(0, 3);
  lcd.print("B4:");
  lcd.print(t4);
  lcd.print("s");

return ;
}

void BarrierTrigered(int i, double &t) {
    if (photoResPrevStates[i] == LOW && photoResStates[i] == HIGH && t == 0) {
      t = ((double)millis() - t_inicial) / 1000.0;  // Tiempo transcurrido tras pasar barrera
      digitalWrite(laserPins[i], LOW);
    }
  // Placeholder for future use
}
