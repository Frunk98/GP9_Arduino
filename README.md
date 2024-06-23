# GP9 Arduino
Bibliotecas de Arduino y software necesario para la IMU GP9 AHRS de CHRobotics.

![GP9 AHRS](https://github.com/Frunk98/GP9_Arduino/blob/main/Imagenes/gp9.png)

## Requisitos  
- **Redshift Serial Interface**: [Descargar aquí](https://www.pololu.com/file/0J1934/SerialInterface_V3-1-5_8-08-2018.zip)

Este software permite la conexión entre el dispositivo GP9 y la PC mediante serial (Es necesario un convertidor de USB a TTL). Permite ajustar la frecuencia de actualización de los datos de los sensores, el Baudrate para la comunicación entre el dispositivo y la computadora, así como algunas calibraciones.

*Baudrates*

![Baudrates](https://github.com/Frunk98/GP9_Arduino/blob/main/Imagenes/rs2.png)

*Frecuencia de datos*

![Frecuencias](https://github.com/Frunk98/GP9_Arduino/blob/main/Imagenes/FR.png)

*Calibración*

![Calibración](https://github.com/Frunk98/GP9_Arduino/blob/main/Imagenes/cal.png)

- **Arduino IDE**: [Descargar aquí](https://github.com/Frunk98/GP9_Arduino/blob/main/FR.png)

Elige la versión del IDE según el sistema operativo.

![SO](https://github.com/Frunk98/GP9_Arduino/blob/main/Imagenes/ard.png)

## Instrucciones de instalación

- **Redshift Serial Interface**: La interfaz del fabricante solo se descomprime y se instala sin ningún paso adicional (Solo para Windows). Para conectar la GP9 por USB, puede utilizarse cualquier adaptador de USB a TTL. Puedes encontrar uno [aquí](https://a.co/d/9Ex6gT6).

- **Arduino IDE**: Para instalar el IDE de Arduino, sigue los pasos indicados en la [página oficial](https://www.arduino.cc/en/software).

- **Librerías GP9**: Para instalar las librerías, comprime los archivos disponibles [aquí](https://github.com/Frunk98/GP9_Arduino/tree/main/GP9-modificados) en un archivo ZIP. Luego, sigue las instrucciones del siguiente tutorial para agregarlas: [Tutorial de instalación de librerías en Arduino](https://www.youtube.com/watch?v=CK1THPvw77M&t=343s).

## __**NOTA**__

A continuación, se presenta una comparación entre las direcciones hexadecimales de cada registro y las variables asociadas, contrastando su disposición original con la forma en que deben ser utilizadas para Arduino:

<div style="display: flex; flex-direction: column; align-items: center;">
    <!-- Primera imagen con pie de imagen -->
    <div style="margin-bottom: 20px;">
        <img src="https://github.com/Frunk98/GP9_Arduino/blob/main/Imagenes/datas.png" alt="Datasheet" style="width: 600px;" />
        <p style="text-align: center;">Datasheet con las direcciones originales</p>
    </div>
</div>
<br>
<pre>
void GP9::save() {
  switch (address) {
    case DREG_HEALTH :
        {
          sats_used = (uint8_t)((data[0] & 0xFC) >> 2);
          hdop = (uint16_t)(((data[0] & 0x03) << 8) | data[1]);
          sats_in_view = (uint8_t)((data[2] & 0xFC) >> 2);
          ovf = (uint8_t)((data[2] >> 1) & 0x01);
          gps_st = (uint8_t)((data[3] & 0x60) >> 5);
          press = (uint8_t)((data[3] >> 4) & 0x01);
          accel = (uint8_t)((data[3] >> 3) & 0x01);
          gyro = (uint8_t)((data[3] >> 2) & 0x01);
          mag = (uint8_t)((data[3] >> 1) & 0x01);
          gps = (uint8_t)(data[3] & 0x01);
        break;  
    }
        case DREG_GYRO_RAW_XY:
        {
          gyro_raw_x = ((int16_t)data[0] << 8) + ((int16_t)data[1] << 8);
          gyro_raw_y = ((int16_t)data[2] << 8) + ((int16_t)data[3] << 8);
          gyro_raw_z = ((int16_t)data[4] << 8) + ((int16_t)data[5] << 8);
          gyro_raw_time = read_register_as_float(6);
            break;
        }
        case DREG_ACCEL_RAW_XY:
        {
              accel_raw_x = ((int16_t)data[0] << 8) + ((int16_t)data[1] << 8);
              accel_raw_y = ((int16_t)data[2] << 8) + ((int16_t)data[3] << 8);
              accel_raw_z = ((int16_t)data[4] << 8) + ((int16_t)data[5] << 8);
          accel_raw_time = read_register_as_float(6);
            break;
        }
}</pre>
<p style="text-align: center;">Código con las direcciones corregidas</p>

<p>Se adjuntan también las bibliotecas originales.
Puedes encontrarlas en este enlace: <a href="https://github.com/Frunk98/GP9_Arduino">GP9-Originales</a>. Las bibliotecas modificadas solo contienen los directorios y variables necesarias para mi proyecto.</p>

## Ejemplos

Este ejemplo lee los datos del giroscopio en los ejes X, Y y Z.

<pre>
#include <Arduino.h>              // Librería de Arduino
#include <GP94.h>                 // Librería local de la GP9

HardwareSerial SerialObject(0); // Define a HardwareSerial object for Serial 0
GP9 imu(SerialObject);           // Initialize the GP9 object with the SerialObject

void setup() {
  Serial.begin(115200);           //Inicializa el Serial con el Baudrate deseado
}

void loop() {
  if (imu.decode(Serial.read())) {  // Función decode de la librería GP9 
    Serial.print(imu.gyro_x); Serial.print(", "); // Datos inerciales
    Serial.print(imu.gyro_y); Serial.print(", ");
    Serial.println(imu.gyro_z);
    delay(1000);                  // Espera de 1 segundo entre lecturas
  }
}</pre>

Primero se incluye la librería que se agregó localmente

<pre>
#include <GP94.h>            
</pre>

Se crea el objeto imu

<pre>
GP9 imu(Serial);               
</pre>

Inicializa el estado y el puerto serie. (La GP9 tiene este Baurate por default)

<pre>
Serial.begin(115200);                
</pre>

Esta función determinará qué registro se está leyendo, la longitud del batch y llamará al checksum una vez que haya terminado. "decode()" devuelve true si se leyó correctamente un paquete.

<pre>
imu.decode(Serial.read())              
</pre>

Se mandan a llamar las variables colocando el objeto imu. antes de cada variable.

<pre>
Serial.print(imu.gyro_x);
Serial.print(imu.gyro_y); 
Serial.println(imu.gyro_z);        
</pre>

__**<p>La lista completa de variables se encuentra disponible <a href="https://github.com/Frunk98/GP9_Arduino/blob/main/GP9-original/GP94.cpp">aquí</a>. Para revisar cuáles son las variables que efectivamente lee el sensor, consulta la <a href="https://github.com/Frunk98/GP9_Arduino/blob/main/Docs/GP9_datasheet.pdf">datasheet</a>.</p>**__

## Resultados

Como se puede observar en la imagen, se están leyendo los datos del giroscopio en los tres ejes, con una frecuencia de 1Hz.

![Resultados](https://github.com/Frunk98/GP9_Arduino/blob/main/Imagenes/Res.png)
