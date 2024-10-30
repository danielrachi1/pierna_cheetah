# Notas del Proyecto

_Apuntes sobre el desarrollo del proyecto_

### 2024-10-07

- En el proyecto PAI utilizaron ROS1. Sería una excelente oportunidad para migrar a ROS2, lo que podría mejorar la eficiencia y actualizar la infraestructura tecnológica del proyecto.
- Parece que se perdió parte del código.Juan Olaya, quien trabajó en el proyecto PAI, mencionó que olvidó hacer el push de la versión más reciente que tenía en la Raspberry Pi al repositorio. Esto refuerza aún más la idea de migrar a ROS2.
- El informe del proyecto PAI parece ser una muy buena fuente de documentación para volver a poner en funcionamiento la pata robótica.

### 2024-10-08

- El código de la pata no estaba en el repositorio de GitHub del equipo de PAI que desarrolló ese proyecto. Afortunadamente, toda la programación se recuperó de la memoria SD de la Raspberry Pi que me entregaron junto con la pata.

### 2024-10-23

- Se confirmaron pérdidas de archivos del proyecto original. El informe de PAI menciona un archivo llamado `CAN_COM.cpp`, pero no lo encontré en los datos descargados de la tarjeta SD. Es posible que el archivo esté en otra parte o que la información que tengo esté desactualizada.
- Juan Olaya me sugirió usar tarjetas ESP32 en lugar de Arduino por temas de memoria y capacidad.
- Como no puedo confiar plenamente en la relación entre el informe de PAI y el código recuperado de la SD, lo más conveniente sería comenzar casi desde cero. Estas son las tareas que me asigno:
  1. Instalar Ubuntu 24.04 en la Raspberry Pi y configurar ROS2 (distribución Jazzy).
  2. Investigar el funcionamiento de los módulos CAN con ESP32 en lugar de Arduino.
  3. Comprar las tarjetas ESP32 o seguir con Arduino, dependiendo de los resultados de la investigación.
  4. Realizar pruebas básicas para entender el funcionamiento del sistema:
     - Leer las posiciones de los motores.
     - Controlar los motores desde ROS2 (prueba básica, sin un control complejo por el momento).
  5. Implementar un control detallado, posiblemente reutilizando los valores K del proyecto PAI.
  6. Desarrollar una interfaz para mover cada eslabón de la pata. Puede ser software o física (me inclinaría más por una interfaz física).
  Completar todo esto implicaría realizar pruebas, lo cual es parte de la Fase 3 de mi proyecto. Una vez completado, discutiré con el profesor Ricardo sobre las guías de laboratorio para poder empezar a escribirlas.
- Para el desarrollo de los nodos de ROS2 usaré Ubuntu Desktop 24.04 en mi computador y Ubuntu Server 24.04 en la Raspberry pi. El flujo de trabajo será el siguiente:
  1. Desarrollar en mi computador personal: Escribir código, mantener control de versiones con git, etc.
  2. "Cross-compile" el programa desde mi computador hacia la arquitectura ARM de la Raspberry.
  3. Copiar los ejecutables compilados a través de SSH / scp.
  4. Conectarme a través de ssh a la raspberry para iniciar los nodos de ROS2.
  5. Iterar.
- Juan Olaya, que trabajó en el proyecto de PAI relacaionado a la pata cheetah, me explicó que sería mejor utilizar una tarjeta ESP32 en vez de arduino por cuestiones de memoria. Revisé que la tarjeta pudiera realizar la comunicación CAN y compré una por mercadolibre para empezar a hacer pruebas.

### 2024-10-24

- Teniendo otra vez la memoria de la Raspberry Pi, intenté encontrar el archivo `CAN_COM.cpp` usando utilidades de la línea de comandos. No se encontró, por lo que confirmo que se perdieron algunos archivos; o que la documentación no es muy precisa / se dejaron algunos detalles sin actualizar.
- Tras revisar los contenidos de la memoria una ultima vez, y asegurarme que se hice correctamente la copia de seguridad, procedo a formatearla para instalar Ubuntu Server 24.04, y ROS2 Jazzy Jalisco.
- Algunos datos respecto a la configuración de la Raspberry Pi:
  - Username: cheetahpi
  - Password: **** (enviar correo a dramirezch@unal.edu.co pidiendo la contraseña)
  - Hostname: cheetahpi.local
  Por lo tanto, para conectarse a la RPi via SSH, se usa el comando: `ssh cheetahpi@cheetahpi.local`. Estando conectado a la misma red WiFi que la RPi.
- Instalé ROS2 Jazzy Jalisco en la RPi, siguiendo la [guía de instalación](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html#resources). No instalé las dev tools, porque todo el desarrollo lo haré en mi computador, no en la RPi. ROS-Base Install, no desktop.

### 2024-10-28

- Logré cargar un programa básico al microcontrolador ESP32. Lo que hice es lo que se puede ver en [este tutorial](https://esp32io.com/tutorials/esp32-hello-world). Encontré algunos problemas en el camino:
  - La conexión debe ser USB tipo A (en el computador) a USB tipo C (en el microcontrolador). Si se usa un cable tipo-c a tipo-c, el micro no se encenderá.
  - El puerto USB-C de la tarjeta de expansión de 30 pines es solo para alimentación. Si se quiere programar el micro se debe conectar directamente al puerto USB-C de la ESP32.
  - Instalé y preparé Arduino IDE siguiendo los siguientes articulos: 
    - [Downloading and installing the Arduino IDE 2](https://docs.arduino.cc/software/ide-v2/tutorials/getting-started/ide-v2-downloading-and-installing/)
    - [ESP32 - Software Instalation](https://esp32io.com/tutorials/esp32-software-installation)

### 2024-10-29

- Hablé con Juan Olaya y me explicó que la pata no tiene un control bien definido. Los movimientos realizados para el proyecto de PAI fueron hechos como rutinas predefinidas. Realizar un sistema de control más robusto debe ser una de mis principales prioridades para este proyecto.
- Es prudente establecer una red VPN para conectarse al servidor del laboratorio. El principal proposito de esto sería poder enviar código a la instancia de GitLab sin tener que estar en la misma red WiFi que el laboratorio.