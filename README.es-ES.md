

# Cardputer GPS Info  

Cardputer GPS Info es una herramienta ligera de información GPS para el M5Stack Cardputer.  
Lee datos GPS a través de UART y muestra la ubicación, velocidad, rumbo, visibilidad de satélites y mapas del cielo directamente en la pantalla del Cardputer.

<img src="https://raw.githubusercontent.com/alcor55/Cardputer-GPS-Info/main/1.jpeg" alt="Screenshot 1" width="30%">

## Changelog
1.2.1 - Verificación del controlador de teclado ADV antes de elegir el driver.
      - Pantalla principal sin microSD e información de arranque en pantalla y USB.

Actualización del paquete 1.2.0 (2026-11-02) - Un único binario completo para M5Burner y M5Launcher.

1.2.0 - Se agregaron perfiles Grove, Cap LoRa-1262 y Ares y configuración manual.
      - Se corrigieron el teclado, las ventanas superpuestas, los conteos y los datos obsoletos.
      - Se agregaron baudios USB configurables, mapa ampliado y barras por constelación.
      - Se agregaron la licencia Apache-2.0, configuración de compilación y pruebas de regresión.
1.1.0 - Se agregó soporte para archivos de configuración en la microSD (cpGpsInfo.conf) para almacenar los pines RX/TX del GPS y la velocidad de baudios.
      - Se agregó la posibilidad de modificar la velocidad de baudios del GPS.
      - Corrección de errores menores.
      - Se agregó el estado de error del GPS (on,off,err).
1.0.0 - Primera versión.

## ToDo
- Validar el perfil GPS Grove con hardware físico cuando haya un módulo disponible.

## Características

- Mostrar datos GPS: Latitud, Longitud, Altitud, Velocidad, Rumbo, Fecha, Hora, HDOP
- Rastrear todos los satélites vistos, los actualmente visibles y los utilizados en la solución
- Mapa del cielo de satélites con estado codificado por colores:
  - Verde: utilizado en la solución
  - Amarillo: visible
  - Rojo: con posición anterior, ya no visible
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

## Instalación y compilación

Ejecuta `pio run` para generar
`build/release/Cardputer_GPS_Info-1.2.1-universal.bin`. Este único archivo incluye
bootloader, tabla de particiones, inicialización OTA y aplicación. Usa el mismo
archivo con M5Burner (dirección **0x0**) o M5Launcher, que extrae la aplicación.
La grabación directa por USB sustituye el launcher instalado. Compilar no graba
el dispositivo. `.pio/build/cardputer/firmware.bin` es un archivo intermedio con
solo la aplicación: no lo publiques en M5Burner ni lo grabes en 0x0.

La aplicación arranca en la pantalla principal incluso sin microSD. Pulsa `c`
para elegir el perfil GPS y después `s` para iniciar la recepción. El arranque
muestra su progreso en pantalla y en USB a 115200 baudios. La versión 1.2.1
requiere verificación física en Cardputer original y ADV con ambos métodos de
instalación; las pruebas ADV de 1.2.0 no validan esta compilación.

## Hardware compatible

- M5Stack Cardputer (1, 1.1, ADV)
- Módulo GPS conectado a pines TX/RX configurables (se configura con la tecla `[c]`)

<img src="https://raw.githubusercontent.com/alcor55/Cardputer-GPS-Info/main/4.jpeg" alt="Screenshot 4" width="30%">

## Frases NMEA a serial USB `[n]`

Si quieres compartir los datos GPS del Cardputer vía serial con otro software, ¡ahora puedes!
Presiona `n`, con el Cardputer conectado vía USB, abre tu software NMEA, selecciona el puerto y ¡disfrútalo!

<img src="https://raw.githubusercontent.com/alcor55/Cardputer-GPS-Info/main/5.jpeg" alt="Screenshot 5" width="50%">

## Imprimir lista de satélites en serial USB `[l]`

<img src="https://raw.githubusercontent.com/alcor55/Cardputer-GPS-Info/main/7.png" alt="Screenshot 7" width="30%">

## Configuración USB y ventanas gráficas

- `c`: configura por separado `GPS baud` y `USB baud`. La velocidad USB se puede
  editar con cualquier perfil y se guarda en la microSD. Los archivos anteriores
  usan 115200 como valor USB predeterminado. En USB CDC nativo este parámetro no
  cambia la velocidad física del enlace USB.
- `f`: abre o cierra un mapa del cielo ampliado en una ventana cuadrada arriba
  a la derecha, con marco verde, sombra negra y sin texto adicional. `p` muestra
  los identificadores y `o` las constelaciones.
- `b`: abre o cierra el gráfico, con un margen de 5 píxeles en cada lado,
  de satélites visibles por constelación, incluidos
  QZSS, SBAS y NavIC. La suma de las barras corresponde a `Visb`.
- Las dos ventanas se actualizan sin detener el GPS. Se puede cambiar entre
  ellas con `f` y `b`, y cerrarlas también con Retroceso.

Las ventanas usan un contenedor común superpuesto a la pantalla principal:
relleno negro, marco verde y borde negro exterior de 2 píxeles. La pantalla
principal permanece visible fuera de la ventana, también al cambiar entre ellas.

## Licencia

Cardputer GPS Info se distribuye bajo la licencia Apache, versión 2.0
(Apache-2.0). Consulta [LICENSE](LICENSE) para leer el texto completo de la
licencia y [NOTICE](NOTICE) para la información de copyright y atribuciones.

Las dependencias de terceros conservan sus respectivas licencias y atribuciones.
