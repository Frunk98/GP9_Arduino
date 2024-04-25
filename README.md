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

<div style="display: flex; flex-direction: column; align-items: center;">
    <h2>Comparación</h2>
    <!-- Primera imagen con pie de imagen -->
    <div style="margin-bottom: 20px;">
        <img src="https://github.com/Frunk98/GP9_Arduino/blob/main/Imagenes/datas.png" alt="Datasheet" style="width: 600px;" />
        <p style="text-align: center;">Datasheet</p>
    </div>
</div>
<pre style="padding: 10px; border: 1px solid #ccc; border-radius: 5px; overflow-x: auto;">
<code style="color: #333; font-size: 14px; font-family: monospace;">
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
}
</code>
</pre>
