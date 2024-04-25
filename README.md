# GP9 Arduino
Bibliotecas de Arduino y sofware necesario para la IMU GP9 AHRS de CHRobotics
## Software necesario 
**Redshift Serial Interface** (https://www.pololu.com/file/0J1934/SerialInterface_V3-1-5_8-08-2018.zip)

Este software permite la conexión entre el dispositivo GP9 y la PC mediante serial (Es necesario un convertidor de USB a TTL) 

![Interfaz gráfica](https://github.com/Frunk98/GP9_Arduino/blob/main/Imagenes/RS.png)

Permite ajustar la frecuencia de actualización de los datos de los sensores, el Baudrate para la comunicación entre el dispositivo y la computadora, así como algunas calibraciónes. 

*Baudrates*

![Baudrates](https://github.com/Frunk98/GP9_Arduino/blob/main/Imagenes/rs2.png)

*Frecuencia de datos*

![Frecuencias](https://github.com/Frunk98/GP9_Arduino/blob/main/Imagenes/FR.png)

*Calibración*

![Calibración](https://github.com/Frunk98/GP9_Arduino/blob/main/Imagenes/cal.png)

**Arduino IDE** (https://github.com/Frunk98/GP9_Arduino/blob/main/FR.png)

Elige la versión del IDE según el sistema operativo.

![SO](https://github.com/Frunk98/GP9_Arduino/blob/main/Imagenes/ard.png)

## Instalaciones necesarias 

**Redshift Serial Interface**
La interface del fabricante solo se descomprime y se instala sin ningún paso adicional. Para conectar la GP9 por UBS puede utilizarse cualquier un adaptador de UBS a TTL (https://a.co/d/9Ex6gT6)

**Arduino IDE*
Para instalar el IDE de Arduino solo sigue los pasos indicados en la página oficial (https://www.arduino.cc/en/software)

**Librerías GP9**
Para instalar las librerías comprime el CPP y el Header en un ZIP y sigue las instrucciones del siguiente tutorial para agregarlas --> (https://www.youtube.com/watch?v=CK1THPvw77M&t=343s) 

