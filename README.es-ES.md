

# Cardputer GPS Info  

Cardputer GPS Info es una herramienta ligera de información GPS para el M5Stack Cardputer.  
Lee datos GPS a través de UART y muestra la ubicación, velocidad, rumbo, visibilidad de satélites y mapas del cielo directamente en la pantalla del Cardputer.

<img src="https://raw.githubusercontent.com/alcor55/Cardputer-GPS-Info/main/1.jpeg" alt="Screenshot 1" width="30%">

## Changelog
1.0.0 - Primera versión.
1.1.0 - Se agregó soporte para archivos de configuración en la microSD (cpGpsInfo.conf) para almacenar los pines RX/TX del GPS y la velocidad de baudios.
      - Se agregó la posibilidad de modificar la velocidad de baudios del GPS.
      - Corrección de errores menores.
      - Se agregó el estado de error del GPS (on,off,err).

## ToDo
- Mejorar el manejo de errores GPS.
- Agregar una ventana emergente de referencia con los pines y la velocidad de baudios para los modelos de GPS más utilizados.

## Características

- Mostrar datos GPS: Latitud, Longitud, Altitud, Velocidad, Rumbo, Fecha, Hora, HDOP
- Rastrear todos los satélites vistos, los actualmente visibles y los utilizados en la solución
- Mapa del cielo de satélites con estado codificado por colores:
  - Verde: utilizado en la solución
  - Amarillo: visible
  - Rojo: no utilizado
- Opcionalmente mostrar la ID del satélite y el sistema en el mapa

<img src="https://raw.githubusercontent.com/alcor55/Cardputer-GPS-Info/main/6.jpeg" alt="Screenshot 1" width="30%">

- Control por teclado:
  - `[s]` Iniciar/Detener serial GPS
  - `[c]` Menú de configuración
  - `[h]` Menú de ayuda
  - `[l]` Imprimir lista de satélites en serial USB
  - `[n]` Imprimir frases NMEA en serial USB
  - `[p]` Mostrar/ocultar ID de satélite en el mapa del cielo
  - `[o]` Mostrar/ocultar sistema en el mapa del cielo

<img src="https://raw.githubusercontent.com/alcor55/Cardputer-GPS-Info/main/2.jpeg" alt="Screenshot 2" width="30%">
<img src="https://raw.githubusercontent.com/alcor55/Cardputer-GPS-Info/main/3.jpeg" alt="Screenshot 3" width="30%">

## Hardware

- M5Stack Cardputer (1, 1.1, ADV)
- Módulo GPS conectado a pines TX/RX configurables (se configura con la tecla `[c]`)

<img src="https://raw.githubusercontent.com/alcor55/Cardputer-GPS-Info/main/4.jpeg" alt="Screenshot 4" width="30%">

## Frases NMEA a serial USB `[n]`

Si quieres compartir los datos GPS del Cardputer vía serial con otro software, ¡ahora puedes!
Presiona `n`, con el Cardputer conectado vía USB, abre tu software NMEA, selecciona el puerto y ¡disfrútalo!

<img src="https://raw.githubusercontent.com/alcor55/Cardputer-GPS-Info/main/5.jpeg" alt="Screenshot 5" width="50%">

## Imprimir lista de satélites en serial USB `[l]`

<img src="https://raw.githubusercontent.com/alcor55/Cardputer-GPS-Info/main/7.png" alt="Screenshot 7" width="30%">
