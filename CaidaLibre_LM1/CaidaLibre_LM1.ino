/* ===== EXPERIMENTO CAIDA LIBRE ===== */
// Codigo utilizado en el experimento de caida libre diseñado para 
// el curso Laboratorio 1: Mecanica I del segundo (2°) semestre de la
// carrera de Ciencias Fisicas - FCFM - Universidad de Concepcion.

/* Librerias */
#include "Adafruit_VL53L0X.h" //Libreria para sensor ToF 

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/* ============================= */

const int BAUDRATE = 9600;

LiquidCrystal_I2C lcd(0x27, 20, 4);

double t_inicial = 0;
double t1, t2, t3, t4 = 0 ; 
bool start = 1;


const int laserPins = {2};         // Pines conectados a los láseres
const int photoResPins = {3};      // Pines conectados a las fotoresistencias
const int button = 4;                // Pin de reset de la barrera

//const int numPairs = sizeof(laserPins);    // Numero de modulos


void setup() {
  Serial.begin(BAUDRATE);

  lcd.init();
  lcd.backlight();

  t_inicial = millis();

  // Initialize pins
  //for (int i = 0; i < numPairs; ++i);
  pinMode(laserPins, OUTPUT);
  digitalWrite(laserPins, HIGH);

  pinMode(photoResPins, INPUT_PULLUP);
  pinMode(button, INPUT);

}

void loop() {

  TemplateDisplay();

  bool pressed = digitalRead(button);
  //Serial.println(pressed);  //Debug

  bool value = digitalRead(photoResPins);
  //Serial.println(value);  //Debug 

  if (pressed) {
    t1 = 0 ; 
    t_inicial = (double)millis();
    digitalWrite(laserPins, HIGH);
    value = digitalRead(photoResPins);  //Si se desalinea, esta variable se mantendra TRUE.
    lcd.setCursor(3, 0);
    lcd.print(t1);
    lcd.print("s");
    //Serial.println(value);            //Debug
    start = 1;
  }

  lcd.setCursor(3, 0);
  lcd.print(t1);
  lcd.print("s");
  
  if (value && start ) {
    t1 = (double)millis() - t_inicial; // Tiempo transcurrido tras pasar primera barrera
    t1 = t1/1000;
    Serial.print("Tiempo en que activo la barrera en segundos: ");
    Serial.println(t1); 
    start = 0;
    digitalWrite(laserPins, LOW);
  }


//============ This could be a function =================
  lcd.setCursor(3, 1);
  lcd.print(t2);
  //lcd.print(" +- 0.005 [s]");

  lcd.setCursor(3, 2);
  lcd.print(t3);
  //lcd.print(" +- 0.005 [s]");

  lcd.setCursor(3, 3);
  lcd.print(t4);
  //lcd.print(" +- 0.005 [s]");
  
}


//============ Funciones (Try objects when you can.) ===========//

void TemplateDisplay() {
  lcd.setCursor(0, 0);
  lcd.print("B1:");
  lcd.setCursor(0, 1);
  lcd.print("B2:");
  lcd.setCursor(0, 2);
  lcd.print("B3:");
  lcd.setCursor(0, 3);
  lcd.print("B4:");
  return 0;
}
