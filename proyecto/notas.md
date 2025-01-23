# Notas del Proyecto

_Apuntes sobre el desarrollo del proyecto_

### 2024-10-07

En el proyecto de PAI utilizaron ROS1. Sería una buena oportunidad para migrar a ROS2.

Parece que se perdió parte del código. Juan Olaya, quien trabajó en el proyecto PAI, mencionó que olvidó hacer el push de la versión más reciente que tenía en la Raspberry Pi al repositorio. Esto refuerza aún más la idea de migrar a ROS2.

El informe del proyecto PAI parece ser una muy buena fuente de documentación para volver a poner en funcionamiento la pata robótica.

### 2024-10-08

El código de la pata no estaba en el repositorio de GitHub del equipo de PAI que desarrolló ese proyecto. Se recuperó la programación de ROS de la memoria SD de la Raspberry Pi que me entregaron junto con la pata.

### 2024-10-23

Se confirmaron pérdidas de archivos del proyecto original. El informe de PAI menciona un archivo llamado `CAN_COM.cpp`, pero no lo encontré en los datos descargados de la tarjeta SD. Es posible que el archivo esté en otra parte o que la información que tengo esté desactualizada.

Juan Olaya me sugirió usar tarjetas ESP32 en lugar de Arduino por temas de memoria y capacidad.

Como no puedo confiar plenamente en la relación entre el informe de PAI y el código recuperado de la SD, lo más conveniente sería comenzar casi desde cero. Estas son las tareas que me asigno:
  1. Instalar Ubuntu 24.04 en la Raspberry Pi y configurar ROS2 (distribución Jazzy).
  2. Investigar el funcionamiento de los módulos CAN con ESP32 en lugar de Arduino.
  3. Comprar las tarjetas ESP32 o seguir con Arduino, dependiendo de los resultados de la investigación.
  4. Realizar pruebas básicas para entender el funcionamiento del sistema: Leer las posiciones de los motores, y Controlar los motores desde ROS2 (prueba básica, sin un control complejo por el momento).
  5. Implementar un control detallado, posiblemente reutilizando los valores K del proyecto PAI.
  6. Desarrollar una interfaz para mover cada eslabón de la pata. Puede ser software o física (me inclinaría más por una interfaz física).
  Completar todo esto implicaría realizar pruebas, lo cual es parte de la Fase 3 de mi proyecto. Una vez completado, discutiré con el profesor Ricardo sobre las guías de laboratorio para poder empezar a escribirlas.

Para el desarrollo de los nodos de ROS2 usaré Ubuntu Desktop 24.04 en mi computador y Ubuntu Server 24.04 en la Raspberry pi. El flujo de trabajo será el siguiente:
  1. Desarrollar en mi computador personal: Escribir código, mantener control de versiones con Git, etc.
  2. "Cross-compile" el programa desde mi computador hacia la arquitectura ARM de la Raspberry.
  3. Copiar los ejecutables compilados a través de SSH / scp.
  4. Conectarme a través de SSH a la Raspberry para iniciar los nodos de ROS2.
  5. Iterar.

Juan Olaya, me explicó que sería mejor utilizar una tarjeta ESP32 en vez de Arduino por cuestiones de memoria. Revisé que la tarjeta pudiera realizar la comunicación CAN y compré una por MercadoLibre para empezar a hacer pruebas.

### 2024-10-24

Teniendo otra vez la memoria de la Raspberry Pi, intenté encontrar el archivo `CAN_COM.cpp` usando utilidades de la línea de comandos. No se encontró, por lo que confirmo que se perdieron algunos archivos; o que la documentación no es muy precisa / se dejaron algunos detalles sin actualizar.

Tras revisar los contenidos de la memoria una última vez, y asegurarme que hice correctamente la copia de seguridad, procedo a formatearla para instalar Ubuntu Server 24.04, y ROS2 Jazzy Jalisco.

Algunos datos respecto a la configuración de la Raspberry Pi:
  
Username: cheetahpi
Password: **** (enviar correo a dramirezch@unal.edu.co pidiendo la contraseña)
Hostname: cheetahpi.local

Por lo tanto, para conectarse a la RPi vía SSH, se usa el comando: `ssh cheetahpi@cheetahpi.local`. Estando conectado a la misma red wifi que la RPi.

Instalé ROS2 Jazzy Jalisco en la RPi, siguiendo la [guía de instalación](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html#resources). No instalé las dev tools, porque todo el desarrollo lo haré en mi computador, no en la RPi. ROS-Base Install, no desktop.

Se compró una [Tarjeta De Desarrollo Esp32 Tipo C](https://articulo.mercadolibre.com.co/MCO-1325137997-tarjeta-de-desarrollo-esp32-tipo-c-_JM) con su respectiva [Board Placa De Expansion Esp32 30 Pines](https://articulo.mercadolibre.com.co/MCO-1400089657-board-placa-de-expansion-esp32-30-pines-_JM). Costaron $28.500 y $14.000 COP respectivamente. $7,800 de envío.

### 2024-10-28

Logré cargar un programa básico al microcontrolador ESP32. Descrito en [este tutorial](https://esp32io.com/tutorials/esp32-hello-world). Encontré algunos problemas en el camino:
  
La conexión debe ser USB tipo A (en el computador) a USB tipo C (en el microcontrolador). Si se usa un cable tipo-c a tipo-c, el micro no se encenderá.
  
El puerto USB-C de la tarjeta de expansión de 30 pines es solo para alimentación. Si se quiere programar el micro se debe conectar directamente al puerto USB-C de la ESP32.
  
Instalé y preparé Arduino IDE siguiendo los siguientes artículos: 
    
[Downloading and installing the Arduino IDE 2](https://docs.arduino.cc/software/ide-v2/tutorials/getting-started/ide-v2-downloading-and-installing/)
    
[ESP32 - Software Instalation](https://esp32io.com/tutorials/esp32-software-installation)

### 2024-10-29

Hablé con Juan Olaya y me explicó que la pata no tiene un control bien definido. Los movimientos realizados para el proyecto de PAI fueron hechos como rutinas predefinidas. Realizar un sistema de control más robusto debe ser una de mis principales prioridades para este proyecto.

### 2024-10-30

Los módulos CAN que se entregaron con la pata son MCP2515_CAN.

Encontré [esta librería](https://github.com/dedalqq/esp32-mcp2515) que proporciona una interfaz de comunicación entre la ESP32 y los módulos MCP2515. También está [esta librería](https://github.com/autowp/arduino-mcp2515) que es más popular, pero no específicamente para ESP32.

Conecté la fuente de alimentación y encendí los motores. Todos parecen estar bien conectados y recibiendo los 24v que necesitan.

Como la conexión de 24v ya está lista, no será necesario hacer el montaje para conectar los motores a 5v. Esto se hace cuando se quieren programar o actualizar el firmware. Pero en uno de los documentos se indica que al alimentar por 24v se puede hacer esto, y se recomienda no conectar la fuente de 24v y la de 5v a la vez.

El ESP32 da señales a 3v, pero parece que los MCP2515 [funcionan a 5 V](https://ww1.microchip.com/downloads/en/DeviceDoc/MCP2515-Stand-Alone-CAN-Controller-with-SPI-20001801J.pdf). He visto algunos tutoriales que usan un MCP2515 con una ESP32, por lo que puede que la diferencia de voltaje no sea un problema, pero me gustaría tener seguridad de que estoy construyendo un sistema robusto. Otra razón por la que no quiero usar el módulo MCP2515, es porque incluye un controlador CAN. La ESP32 también incluye un controlador CAN. Lo que necesita la ESP32 es un CAN transceiver.

Inicialmente, me incliné por usar conversores CAN [TJA1050](https://www.nxp.com/docs/en/data-sheet/TJA1050.pdf) en vez de los MCP2515. Los 1050 son solo  transceivers. Sin embargo, no logré encontrar una respuesta definitiva a si este transceiver es compatible con dispositivos cuyos pines lógicos son de 3 V, como es el caso de la ESP32.

Investigué otros transceivers diseñados para 3 V. De los que encontré, el único que se puede conseguir en Colombia es el [Sn65hvd230](https://www.ti.com/lit/ds/symlink/sn65hvd232.pdf?ts=1730758637703&ref_url=https%253A%252F%252Fwww.google.com%252F). Este cumple con las especificaciones que estoy buscando: alimentación y lógica de 3V, transceiver CAN sin control. Sin embargo, no encontré muy buenas referencias respecto a este chip. Encontré en varios foros personas con varios problemas sin solución; o que la solución fue cambiar de chip. Y como los integrados que se consiguen en Colombia son generalmente genéricos, es probable que yo también vaya a tener problemas. Esto me deja con dos opciones:

1. Arriesgarme con los Sn65hvd230.
2. Utilizar un Arduino, sabiendo que puede que me dé problemas por su poca memoria.

Intentaré utilizar los Sn65hvd230. Aunque no haya encontrado muy buenas referencias, cumplen con la especificación que estoy buscando, por lo que me gustaría ver de primera mano su calidad. Si estos no funcionan, tendré que utilizar un Arduino en vez de la ESP32. Los otros transceivers CAN no se consiguen muy fácilmente, por lo que tendría que importarlos, y esto retrasaría demasiado mi avance en el proyecto.

### 2024-11-05

Se compraron 4 [Can Bus Sn65hvd230](https://articulo.mercadolibre.com.co/MCO-630269980-can-bus-sn65hvd230-_JM) costaron $12,000 cada una y  $7,425 de envío. Compré 4 para tener de sobra en caso de que alguno se dañe. Pero espero poder hacer el montaje del robot con uno solo.

### 2024-11-06

Realicé el montaje descrito en `prototipos/can_basico/`. Conecté los circuitos pero no logré hacer que cumpliera la función que quiero. Intenté:

1. Utilizar los ejemplos de la librería [arduino-CAN](https://github.com/sandeepmistry/arduino-CAN). Esto no funcionó porque actualmente hay un error que evita que la librería compile para la ESP32: `esp_intr.h: No such file or directory`. Parece haber un workaround que implica instalar una version anterior del firmware de la ESP32, pero no creo que esto sea adecuado ya que hay que bloquear actualizaciones automaticas de todo el Arduino IDE.
2. Seguir utilizando la librería arduino-CAN para el arduino (sender), pero usar la librería [ESP32-TWAI-CAN](https://docs.arduino.cc/libraries/esp32-twai-can/) para el ESP32 (reciever). Esto compiló, pero no logré leer los mensajes enviados por el Arduino.

### 2024-11-07

Seguí intentando hacer funcionar el experimento de ayer. Intenté buscar más librerías, busqué souciones a los problemas que me estaban dando las librerías, y verifiqué que todas las conexiones estuvieran en buen estado.

No encontré ninguna solución, así que decidí cambiar lo que estaba intentando.

En vez de usar Arduino IDE para programar la ESP32, decidí usar el Framework oficial que mantiene el fabricante del chip ESP32: [Espressif ESP-IDF](https://idf.espressif.com/).

Espressif incluye varios programas de ejemplo, uno de ellos es: [TWAI Self Test Example](https://github.com/espressif/esp-idf/tree/v5.2.3/examples/peripherals/twai/twai_self_test). Como indica la documentación "The Self Test Example can be run as a simple test to determine whether a target (ESP32, ESP32-S2, ESP32-S3 or ESP32-C3) is properly connected to a working external transceiver." Ejecuté este programa en la ESP (conectada al SN65HVD230) y funciona justo como indica el ejemplo. Por lo que puedo estar seguro de que la ESP y el transciever funcinan bien.

### 2024-11-08

Ahora que tengo un código que sé que funciona en la ESP con el transciever, me dedicaré a entenderlo.

### 2024-11-12

Para entender el código del ejemplo, utilicé [este chat](https://chatgpt.com/share/6734db3a-a838-8003-9c52-b4395c54cb12) para explicar/aclarar algunos puntos que no me quedaban muy claros.

Edité el código para que la ESP32 solo recibiera paquetes, pero no funcionó. Como el ejemplo de enviarse paquetes a sí misma sí funciona, es seguro asumir que este es un problema con el Arduino o el MCP2515. 

Ahora que entiendo cómo leer CAN frames desde la ESP32, procederé a entender qué datos puedo intercambiar con los motores. Para luego conectarlos a mi prototipo.

### 2024-11-20

Estuve revisando a fondo algunos articulos de documentación de los motores, principalmente: [el de Bart](https://docs.google.com/document/d/1QIEI6IdHOcW4N1cRyucb33io4LriNYafIMs1sjLfTQU/edit?tab=t.0). Los motores se conocen por varios nombres. MIT cheetah motor es el diseño original que se uso para fabrical el cheetah del MIT, pero estos no son comerciales. SteadyWin es el nombre de la marca que los fabrica y comercializa. HobbyKing es un nombre que se les dío por su calidad casi profesional. (En algunos lados se refiere a HobbyKing como el Mini-Cheetah completo.)

### 2024-11-21

Quiero hacer un programa que haga lo siguiente:

1. Enviar un comando de posición desde mi pc a la esp32.
2. La esp32 lee la info enviada y la formatea para enviarla por can a los motores.
3. Se envía el paquete can a los motores.
4. El motor se mueve a la posición indicada y envía su respuesta por CAN.
5. La esp lee la respuesta y la formatea para mostrarla en consola.
6. La esp espera un nuevo comando de posición.
7. Repetir hasta salir del programa.

Algunos detalles:

- El programa correrá completamente desde la terminal.
- La esp estará conectada por USB y se comunicará por serial al pc.
- [En este articulo](https://docs.google.com/document/d/1dzNVzblz6mqB3eZVEMyi2MtSngALHdgpTaDJIW_BpS4/edit?usp=sharing) se especifica cómo funciona la comunicación CAN con el motor.

Para lograr esto, debo entender cómo:

- Enviar y recibir datos por serial a la ESP32.
- Convertir los datos enviados por serial a un paquete CAN.
- Enviar y recibir paquetes CAN.

Para lo primero, escribí dos programas sencillos. Uno en el IDE de Arduino y el otro usando ESP-IDF. Ambos se pueden encontrar en `experimentos/serial/`. Lo único destacable a mencioar acá es que se debieron hacer algunos [ajustes a la configuración](https://chatgpt.com/share/673fb215-b978-8003-8f16-062069c2e3a0) del ejemplo dado por espressif para que la esp32 usara el puerto serial por USB y no por los pines predeterminados del ejemplo: UART Port Number: 0, UART TX Pin: 1 UART RX Pin: 3.

### 2024-11-22

El siguiente paso es poder convertir paquetes CAN.

En [este archivo](https://os.mbed.com/users/benkatz/code/HKC_MiniCheetah//file/fe5056ac6740/CAN/CAN_com.cpp/) se especifica a detalle la estructura de los paquetes CAN que recibe y envía el motor.

#### Estructura del Paquete de Respuesta CAN

- **Posición** de 16 bits, entre -4π y 4π
- **Velocidad** de 12 bits, entre -30 y +30 rad/s
- **Corriente** de 12 bits, entre -40 y 40

El paquete CAN consta de 5 palabras de 8 bits.

Formateado de la siguiente manera. Para cada cantidad, el bit 0 es el LSB (bit menos significativo).

| Byte | Bits en el byte | Contenido              | Bits de la cantidad      |
|------|-----------------|------------------------|--------------------------|
| 0    | 7-0             | posición\[15-8\]       | bits 15-8 de posición    |
| 1    | 7-0             | posición\[7-0\]        | bits 7-0 de posición     |
| 2    | 7-0             | velocidad\[11-4\]      | bits 11-4 de velocidad   |
| 3    | 7-4             | corriente\[11-8\]      | bits 11-8 de corriente   |
| 3    | 3-0             | velocidad\[3-0\]       | bits 3-0 de velocidad    |
| 4    | 7-0             | corriente\[7-0\]       | bits 7-0 de corriente    |

#### Estructura del Paquete de Comando CAN

- **Comando de posición** de 16 bits, entre -4π y 4π
- **Comando de velocidad** de 12 bits, entre -30 y +30 rad/s
- **kp** de 12 bits, entre 0 y 500 N·m/rad
- **kd** de 12 bits, entre 0 y 100 N·m·s/rad
- **feed-forward torque** de 12 bits, entre -18 y 18 N·m

El paquete CAN consta de 8 palabras de 8 bits.

Formateado de la siguiente manera. Para cada cantidad, el bit 0 es el LSB (bit menos significativo).

| Byte | Bits en el byte | Contenido              | Bits de la cantidad         |
|------|-----------------|------------------------|-----------------------------|
| 0    | 7-0             | posición\[15-8\]       | bits 15-8 de posición cmd   |
| 1    | 7-0             | posición\[7-0\]        | bits 7-0 de posición cmd    |
| 2    | 7-0             | velocidad\[11-4\]      | bits 11-4 de velocidad cmd  |
| 3    | 7-4             | kp\[11-8\]             | bits 11-8 de kp             |
| 3    | 3-0             | velocidad\[3-0\]       | bits 3-0 de velocidad cmd   |
| 4    | 7-0             | kp\[7-0\]              | bits 7-0 de kp              |
| 5    | 7-0             | kd\[11-4\]             | bits 11-4 de kd             |
| 6    | 7-4             | tff\[11-8\]            | bits 11-8 de tff            |
| 6    | 3-0             | kd\[3-0\]              | bits 3-0 de kd              |
| 7    | 7-0             | tff\[7-0\]             | bits 7-0 de tff             |

Escribí algunas funciones que ayudarán a manejar esta información en: `experimentos/can_utils/`.

### 2024-11-25

El siguiente paso sería entender cómo enviar y recibir paquetes CAN con la ESP32. Pero esto es algo que ya he hecho. El código para esto se puede encontrar en el [TWAI self test example](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/twai/twai_self_test).

Para ponerlo todo junto, escribiré el programa propuesto en `experimentos/single_motor_control/`.

Para empezar, escribí agunas funciones para ayudarme con la comunicación serial UART. Correr unit tests fue más dificil de lo que esperaba, así que acá dejo el proceso:

1. Escribir los tests. Conectar la ESP32.
2. En VSCode, con la extensión de ESP-IDF instalada y configurada. `ctrl+shift+p` y buscar la opción: `ESP-IDF: Unit test: Install ESP-IDF pytests requirements.` Eso se debe hacer una sola vez.
3. Barra lateral izquierda de VSCode > testing > refresh tests. (el simbolo de recarga en la parte superior.)
4. Desde esa misma barra lateral, run tests.

Si el build and flash del paso 3 falla, probablemente sea porque tienes alguna terminal leyendo datos de la ESP32. Cierra todas las terminales de VSCode y vuelve a darle a refresh tests.

En algunas ocasiones tuve que hacer `ctrl+shift+p` y buscar la opción: `ESP-IDF: Unit test: build and flash unit test app for testing` antes del paso 3. No entiendo muy bien por qué, ya que supuse que el paso 3 incluye este comando. Pero si estás intentando hacer debug de uno de estos pasos, intenta correr el comando.

Cada vez que cambies algún test o agregues nuevos debes darle a refresh tests.

### 2024-11-26

Si se agregan tests nuevos, es buena idea cerrar y volver a abrir VSCode.

Es más facil ver resultados de tests en la pestaña _monitor device_. Sin embargo, este algunas veces falla, y la pestaña no abre. Para volver a hacerlo funcionar lo que hice fue:

1. Build flash and monitor el programa en el que se está trabajando. (No la test app.)
2. Build and flash unit test app for testing.
3. Abrir el monitor.

### 2024-11-27

Conecté la ESP32 al motor 1 de la pata. El programa funcionó y logré enviar comandos básicos al motor. Hice algunos ajustes a cómo la ESP32 interpreta los mensajes de respuesta del motor.

En uno de los primeros comandos enviados, el motor se movió más rápido de lo que esperaba. Lo que llevó a una fractura en el tercer eslabón. Ya estaba en mis planes reemplazar esta pieza, por lo que esto no es un problema grave.

Desacoplé los eslabones 2 y 3. Esto me permitirá realizar pruebas más seguras mientras entiendo correctamente los parámetros de movimiento del motor.

### 2024-11-28

Ahora que sé cómo enviar comandos a los motores, quiero implementar un mejor sistema para controlar el robot. Escribí esta especificación para referencia futura:

Voy a desarrollar un sistema de control robótico utilizando el microcontrolador ESP32 para gestionar un robot de tres grados de libertad (3-DOF). El ESP32 es una opción adecuada debido a su bajo costo, eficiencia energética y capacidades integradas de Wi-Fi y Bluetooth.

El sistema contará con una interfaz gráfica de usuario (GUI) basada en la web, alojada directamente en el ESP32. Esta elección permite a los usuarios supervisar y controlar el robot desde cualquier dispositivo con un navegador web, sin necesidad de hardware adicional. La GUI proporcionará una plataforma intuitiva para interactuar con el robot, facilitando su operación y monitoreo. Esto estará inspirado en Mainsail.

Además, integraré un controlador físico conectado al ESP32 mediante Bluetooth. Esta configuración permite el control manual directo del robot, proporcionando una alternativa al control basado en la web. El ESP32 procesará las entradas tanto de la GUI como del controlador físico, traduciéndolas en comandos precisos para los motores que controlan los movimientos del robot. Esta doble interfaz ofrece flexibilidad, permitiendo al usuario elegir entre el control remoto a través de la interfaz web o el control directo mediante el controlador físico, según las necesidades específicas de cada situación.

Escribí una buena parte del firmware para hacer esto. En este momento se pueden enviar comandos desde una página web, hacer el parsing, y enviarlos al motor via CAN.

Es importante recordar que para que la ESP32 se conecte al WiFi, se deben asignar las credenciales en menuconfig.

### 2024-12-02

No estoy satisfecho con la UI actual. Para mejorarla empezaré por separar el código HTML/CSS/JS del archivo `main.c`. Esto me permitirá poder hacer modificaciones más modulares. Para esto usaré spiffs.

Para que spiffs funcione, es necesario (además de ajustar el código del proyecto):
1. Crear un archivo `partitions.csv` en la carpeta root del proyecto.
2. En menuconfig, indicar que se debe usar el archivo .csv para indicar las particiones.
3. Cambiar el flash size a 4 MB (en menuconfig).


### 2024-12-03

En el laboratorio, con la ESP32 conectada al motor 1, realicé algunas pruebas para 1) verificar que mi firmware esté funcionando bien y 2) encontrar valores adecuados para las ganacias del control.

Verifiqué que el firmware de la ESP32 funciona bien. Puedo acceder la página, enviar comandos, los comandos se traducen y se envían por CAN, y puedo leer los mensajes de respuesta del motor.

Sin embargo, el control no parece estar funcionando bien. Parece haber algún error en la conversión, no sé en que parte. Cuando envío 11 grados, se va a 90, cuando envío 22, va a aprox 160; 33 unos 300. Si lo envío a 90 da varias vueltas.

Creo que el error está en las definiciones de los rangos. En `components/message_parser/include/message_parser.h` están definidos segun [este archivo](https://os.mbed.com/users/benkatz/code/HKC_MiniCheetah//file/fe5056ac6740/CAN/CAN_com.cpp/). En la [documentación del motor drive](https://docs.google.com/document/d/1dzNVzblz6mqB3eZVEMyi2MtSngALHdgpTaDJIW_BpS4/edit?tab=t.0) revisé todos los links de repositorios intentando buscar otros valores para estos límites.

En [este archivo](https://os.mbed.com/users/benkatz/code/CanMaster//file/107df25e1eef/main.cpp/), encontré:

```
#define P_MIN -95.5f
 #define P_MAX 95.5f
 #define V_MIN -45.0f
 #define V_MAX 45.0f
 #define KP_MIN 0.0f
 #define KP_MAX 500.0f
 #define KD_MIN 0.0f
 #define KD_MAX 5.0f
 #define I_MIN -18.0f
 #define I_MAX 18.0f
 ```

 A primera vista, estos valores me podrían servir, los límites de posición son mayores a los que tengo actualmente, lo que indica que los valores se podrían ajustar cómo los necesito (en este momento valores pequeños causan angulos grandes, si tengo límites más grandes, los mismos valores causarán angulos más pequeños).

 Estos valores arreglaron el problema.

 Ahora que puedo mover el motor a algunas posiciones, voy a encontrar valores para velocidad, kp, kd y torque feed forward.

 - Velocidad: Este valor no es la velocidad que tomará el motor para llegar a la posición deseada. Es para hacer control de velocidad. Usaré 0 siempre.
 - Kp: 10
 - Kd: 5
  - Feed-forward torque: considero que es adecuado dejar este valor en 0, por ahora.

Necesito encontrar alguna manera de hacer que tenga un arranque más suave.

### 2024-12-10

Voy a implementar curvas de movimiento. Lo haré creando otro componente en el firmware de la ESP32.

Curvas implementadas en el firmware. Aún debo probarlas en el laboratorio.

Funciona!

Actualmente hay un bug. Por alguna razón, cuando se vuelve a encender el motor, y este no está en posición 0, el primer movimiento lo hace sin suavizado. Esto puede ser peligroso.

### 2024-12-16

Cambié cómo la ESP32 se conecta a la red WiFi y ahora se puede entrar a la página desde `http://cheetah.local`. En caso que esto no funcione, la esp imprime en serial su IP, para acceder directamente desde esta.

Revisé la bolsa de repuestos que estaba en el laboratorio. Encontré varias piezas que me sirven. Ahora solo me hace falta el eslabon final.

La polea tiene un rodamiendo que no se puede remover. Creo que está pegado. Debo diseñal el eslabon final de tal manera que no deba introducirlo en la barra del rodamiento.

### 2024-12-17

Intenté diseñar el eslabón en FreeCAD. Pero la curva de aprendizaje es mayor de lo que esperaba. Por ahora voy a diseñar en fusion (la version online), pero en un futuro se deberían pasar estos modelos a un software open source.

### 2024-12-18

Reparé el cableado de los tres motores. Los cables de alimentación estaban mal soldados y mal aislados, lo que previamente causó un corto circuito durante una prueba. Realicé una nueva soldadura de los conectores y aislé los terminales con tubos termorretráctiles. Aunque los cables de datos no representaban riesgo, eran muy frágiles, por lo que también los protegí con tubos termorretráctiles para reforzar su integridad.

Después de varias iteraciones e impresiones del diseño, logré finalizar el eslabón final. El archivo del diseño está disponible en la carpeta `cad`. La interfaz para el montaje de la herramienta, que utiliza tres tornillos, se define de la siguiente manera: tres agujeros para tornillos M4 dispuestos alrededor de un eje central, donde el centro de cada agujero se encuentra a 6 mm del eje.

### 2025-01-15

Compré bolas de squash que servirán como "zapatos" para la pata.

Decidí que para organizar los componentes y circuitos de manera adecuada, lo mejor es utilizar una placa de MDF como base, que se sostendrá a los perfiles de aluminio inferiores, posiblemente al cuadrante posterior derecho, y que tendrá un layout que definirá los huecos para tornillos que anclarán cada componente. Para esto necesito empezar por tomar medidas de los perfiles en estos cuadrantes.

### 2025-01-20

Diseñé la herramienta que sostendrá la bola de squash.

Para organizar los diferentes componentes electrónicos, cables, etc. Voy a cortar una pieza de MDF que colocaré en el cuadrante opuesto a la pata en el montaje. El layout lo realizo en Inventor.

Diseñé un case para organizar la ESP32 y el módulo CAN. La ESP32 debe estar conectada por USB para que funcione el montaje. Decidí que esto es apropiado ya que por ahora no se planea usar la pata sin un computador.

### 2025-01-22

Diseñé una caja/panel para los 3 switches del montaje (1 de cada motor).

### 2025-01-23

Decidí que no es necesario usar MDF y cortarlo con laser para atornillar los componentes. Puedo usar tornillos para madera.