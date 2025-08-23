/* ===== Time of flight sensor ===== */
// Primer codigo para ToF sensor
// La idea de este codigo es aprender a usarlo para integrarlo mas tarde
// al Arduino. Debe funcionar aquí primero

/* Librerias */

#include "Adafruit_VL53L0X.h" //Libreria para sensor ToF 
#include <Wire.h>

/* ============================= */

const int BAUDRATE = 9600;

// Llamando sensor ToF
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  Serial.begin(BAUDRATE);

  // Initialize sensor
  Serial.println("VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Error initializing VL53L0X"));
    while(1);
  }

}

void loop() {

  VL53L0X_RangingMeasurementData_t measure;  

  Serial.println("Reading sensor... ");
  lox.rangingTest(&measure, false); //if true is passed as a parameter, it shows debug data on the serial port
  
  if (value && start ) {
    if (measure.RangeStatus != 4){
      Serial.print("Distance (mm): ");
      Serial.println(measure.RangeMilliMeter);
    } 
    else{
      Serial.println("  Out of range ");
    }
  }
  
}

