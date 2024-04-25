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

- **Redshift Serial Interface**: La interfaz del fabricante solo se descomprime y se instala sin ningún paso adicional. Para conectar la GP9 por USB, puede utilizarse cualquier adaptador de USB a TTL. Puedes encontrar uno [aquí](https://a.co/d/9Ex6gT6).

- **Arduino IDE**: Para instalar el IDE de Arduino, sigue los pasos indicados en la [página oficial](https://www.arduino.cc/en/software).

- **Librerías GP9**: Para instalar las librerías, comprime los archivos disponibles [aquí](https://github.com/Frunk98/GP9_Arduino/tree/main/GP9-modificados) en un archivo ZIP. Luego, sigue las instrucciones del siguiente tutorial para agregarlas: [Tutorial de instalación de librerías en Arduino](https://www.youtube.com/watch?v=CK1THPvw77M&t=343s).

## Ejemplos

__**NOTA**__

Las direcciones hex. de los registros son las mismas de la datasheet (https://github.com/Frunk98/GP9_Arduino/blob/main/Docs/GP9_datasheet.pdf), sin embargo las variables de los directorios no, 
<div style="display: flex; flex-direction: column; align-items: center;">
    <h2>Comparación</h2>
    <!-- Primera imagen con pie de imagen -->
    <div style="margin-bottom: 20px;">
        <img src="https://github.com/Frunk98/GP9_Arduino/blob/main/Imagenes/datas.png" alt="Datasheet" style="max-width: 50%;" />
        <p style="text-align: center;">Pie de la imagen 1: Datasheet</p>
    </div>
    <!-- Segunda imagen con pie de imagen -->
    <div style="margin-bottom: 20px;">
        <img src="https://github.com/Frunk98/GP9_Arduino/blob/main/Imagenes/var.png" alt="Biblioteca" style="max-width: 50%;" />
        <p style="text-align: center;">Pie de la imagen 2: Biblioteca</p>
    </div>
    <!-- Pie de imagen general -->
    <p style="text-align: center;">Comparación</p>
</div>


