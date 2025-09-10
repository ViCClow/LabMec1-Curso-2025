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

void timeDisplay(int a, int b, double c) ;
void TemplateDisplay();

const int BAUDRATE = 9600;

LiquidCrystal_I2C lcd(0x27, 20, 4);

double t_inicial = 0;
double t1, t2 = 0;
bool start = 1;


int laserPins[] = { 3, 5};           // Pines conectados a los láseres
int photoResPins[] = { 4, 6};   // Pines conectados a las fotoresistencias
int button = 2;                       // Pin de reset de la barrera

int numPairs = sizeof(laserPins)/sizeof(laserPins[0]);  // Numero de modulos
bool photoResStates[sizeof(photoResPins)/sizeof(photoResPins[0])]; // Estados de las fotoresistencias


void setup() {
  Serial.begin(BAUDRATE);

  lcd.init();
  lcd.backlight();

  t_inicial = millis();

  pinMode(button, INPUT_PULLUP);

  // Initialize pins
  for (int i = 0; i < numPairs; ++i) {
    pinMode(laserPins[i], OUTPUT);
    digitalWrite(laserPins[i], HIGH);

    pinMode(photoResPins[i], INPUT);
  }
}

void loop() {

  TemplateDisplay();

  //============ Restart button ============//
  //  For debugging uncomment the Serial.println line
  bool pressed = digitalRead(button); // Standby state = 0
  //Serial.println(pressed); 

  //=========== Read photoresistors ============//
  // For debugging uncomment the Serial.print lines:
  for (int i = 0; i < numPairs; ++i) {
    photoResStates[i] = digitalRead(photoResPins[i]);
    Serial.print("Sensor "); Serial.print(i); Serial.print(": "); Serial.println(photoResStates[i]);
  }

  //=========== Reset ============//
  if (pressed) {
    t1 = 0;
    t2 = 0;

    t_inicial = (double)millis();
    for (int i = 0; i < numPairs; ++i) {
      digitalWrite(laserPins[i], HIGH);
    }
  
    lcd.clear();
    TemplateDisplay();
    start = 1;
  }

  //=========== Measure time ============//
  for (int i = 0; i < numPairs; ++i) {
    if (photoResStates[i] && start) {
      //Serial.print("Sensor "); Serial.print(i); Serial.print(": "); Serial.println(photoResStates[i]);
      if (i == 0 && t1 == 0) {
        t1 = (double)millis() - t_inicial;  // Tiempo transcurrido tras pasar primera barrera
        t1 = t1 / 1000;
      }
      if (i == 1 && t2 == 0) {
        t2 = (double)millis() - t_inicial;  // Tiempo transcurrido tras pasar segunda barrera
        t2 = t2 / 1000;
      }
    
  

    //=========== Deactivate barrier. ===========//
    start = 0;
    for (int j = 0; j < numPairs; ++j) {
      digitalWrite(laserPins[j], LOW);
      }
    }
  }

timeDisplay(0,3,t1);
timeDisplay(1,3,t2);
//timeDisplay(2,3,t3);
//timeDisplay(3,3,t4);

}

//============ Funciones (Try objects when you can.) ===========//

void timeDisplay(int a, int b, double c) {
  lcd.setCursor(a, b);
  lcd.print(c);
  lcd.print("s");

return ;
}

void TemplateDisplay() {

  lcd.setCursor(0, 0);
  lcd.print("B1:");
  lcd.print(t1);
  lcd.print("s");

  lcd.setCursor(0, 1);
  lcd.print("B2:");
  lcd.print(t2);
  lcd.print("s");

  //lcd.setCursor(0, 2);
  //lcd.print("B3:");
  //lcd.print(t3);
  //lcd.print("s");
//
  //lcd.setCursor(0, 3);
  //lcd.print("B4:");
  //lcd.print(t4);
  //lcd.print("s");

return ;
}
