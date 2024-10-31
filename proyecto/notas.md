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