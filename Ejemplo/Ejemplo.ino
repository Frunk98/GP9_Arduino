#include <Arduino.h>              // Librería de Arduino
#include <GP94.h>                 // Librería local de la GP9

GP9 imu(Serial);                  // Objeto del sensor GP9

void setup() {
  Serial.begin(115200);           //Inicializa Baudrate
}

void loop() {
  if (imu.decode(Serial.read())) {  // Función decode de la librería GP9 
    Serial.print(imu.gyro_x); Serial.print(", "); // Datos inerciales
    Serial.print(imu.gyro_y); Serial.print(", ");
    Serial.println(imu.gyro_z);
    delay(1000);                  // Espera de 1 segundo entre lecturas
  }
}