/* ===== EXPERIMENTO CAIDA LIBRE ===== */
// Codigo utilizado en el experimento de caida libre diseñado para
// el curso Laboratorio 1: Mecanica I del segundo (2°) semestre de la
// carrera de Ciencias Fisicas - FCFM - Universidad de Concepcion.

/* Librerias */

#include "VL53L0X.h" //Libreria para sensor ToF

#include <Arduino.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/* ============================= */
void TimeDisplay();

const int BAUDRATE = 9600;

LiquidCrystal_I2C lcd(0x27, 20, 4);
VL53L0X sensor;

double t_inicial = 0;
bool start = 1;

const int laserPins[] = {3, 5, 7, 9};           // Pines conectados a los láseres
const int photoResPins[] = {4, 6, 8, 10};   // Pines conectados a las fotoresistencias
const int magnetPin = 13;                    // Pin del electroiman
const int button = 2;                       // Pin de reset de la barrera

const int numPairs = sizeof(laserPins)/sizeof(laserPins[0]);  // Numero de modulos/barreras

int triggeredBarriers[numPairs] = {-1, -1, -1, -1}; // Barreras que han sido activadas
double triggeredTimes[numPairs] = {0, 0, 0, 0}; // Tiempos en que fueron activadas
int triggerCount = 0; // Contador de barreras activadas

bool photoResStates[sizeof(photoResPins)/sizeof(photoResPins[0])]; // Estados actuales
bool photoResPrevStates[sizeof(photoResPins)/sizeof(photoResPins[0])]; // Estados previos

uint16_t triggeredDistances[numPairs] = {0, 0, 0, 0}; // Distancias medidas por VL53L0X

bool magnetState = false;


void setup() {
  Serial.begin(BAUDRATE);
  Wire.begin();
  // ====== Sensor ToF ====== //
  sensor.init();
  sensor.setTimeout(500);
  // ====== LCD ========//
  lcd.init();
  lcd.backlight();
// =================//

  t_inicial = millis();
  pinMode(button, INPUT_PULLUP);
  pinMode(magnetPin, OUTPUT);
  digitalWrite(magnetPin, LOW); // Inicializa el electroiman apagado

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


  digitalWrite(magnetPin, HIGH);

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

    digitalWrite(magnetPin, LOW);

    for (int i = 0; i < numPairs; ++i) {
      triggeredBarriers[i] = -1;
      triggeredTimes[i] = 0;
    }
    triggerCount = 0;

    t_inicial = (double)millis();
    for (int i = 0; i < numPairs; ++i) {
      digitalWrite(laserPins[i], HIGH);
      photoResPrevStates[i] = photoResStates[i]; //Reinicia estado previo.
    }
  
    lcd.clear();
    TimeDisplay();
  }

  //=========== Measure time ============//

  for (int Barrera = 0; Barrera < numPairs; ++Barrera) {
    //Serial.print("Contador de estados previo: "); Serial.println(photoResStates[Barrera]);
    if (photoResPrevStates[Barrera] == LOW && photoResStates[Barrera] == HIGH) {
      //Serial.print("Contador de estados: "); Serial.println(photoResStates[Barrera]);
      // Add count to change to next barrier.
        triggeredBarriers[triggerCount] = triggerCount;
        
        triggeredTimes[triggerCount] = ((double)millis() - t_inicial) / 1000.0;
        digitalWrite(laserPins[Barrera], LOW);
        triggerCount++;

        // === VL53L0X Single Shot ===
        uint16_t distance = sensor.readRangeSingleMillimeters();
        if (sensor.timeoutOccurred()) {
          Serial.println("VL53L0X TIMEOUT");
        } else {
          Serial.print("VL53L0X Distance (mm): ");
          Serial.println(distance);
          triggeredDistances[triggerCount] = distance;
        }
    }
  }
  
  // ========== Actualizar estados previos. ========== //
  for (int i = 0; i < numPairs; ++i) {
    photoResPrevStates[i] = photoResStates[i];
  }

}

//============ Funciones (Try objects when you can.) ===========//

void TimeDisplay() {

  for (int i = 0; i < triggerCount; ++i) {
    lcd.setCursor(0, i);
    if(triggeredBarriers[i] != -1) {
        lcd.print("B");
        lcd.print(triggeredBarriers[i] + 1);
        lcd.print(":");
        lcd.print(triggeredTimes[i]);
        lcd.print("s");

        lcd.setCursor(10, i);
        lcd.print(triggeredDistances[i]);
        lcd.print("mm");
    } else {
      lcd.print("B-:0.000s         ");
    }
  }
return ;
}


