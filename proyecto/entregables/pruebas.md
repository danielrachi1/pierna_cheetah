# Informe de Pruebas de Funcionamiento

Este documento describe las pruebas que se realizarán para verificar el correcto funcionamiento de la pierna robótica y de los subsistemas involucrados. Estas pruebas abarcan tanto aspectos mecánicos como electrónicos y de software, de modo que se garantice la fiabilidad y seguridad de todo el dispositivo.

## Continuidad de Circuitos

### Propósito

Verificar que no existan cortos, conexiones abiertas o soldaduras defectuosas en la fuente de alimentación, interruptores, motores y transceptores CAN.

### Procedimiento  

1. Usar un multímetro en modo continuidad para revisar todos los cables de alimentación (24 V y 5 V, si aplica).  
2. Revisar el cableado de datos CAN (bus CANH y CANL) entre la ESP32 y cada motor.  
3. Verificar la conexión de los interruptores de emergencia y de los switches individuales de cada motor.

### Criterios de Aceptación

- No debe haber continuidad entre líneas de alimentación y masa (GND) salvo donde corresponda.  
- Las líneas de datos deben mostrar continuidad adecuada al bus CAN (no abiertos ni en corto entre sí).  
- Cada interruptor debe abrir y cerrar el circuito sin falsos contactos.

## Prueba del Control y la Interfaz Web

### Propósito

Asegurar que la interfaz de usuario, accesible vía WiFi, permita enviar comandos de control a los motores y visualizar su estado en tiempo real.

### Procedimiento

1. Ingresar a la página de control alojada en la ESP32 (`http://cheetah.local`) desde un computador o dispositivo móvil.  
2. Enviar un comando de posición al motor 1.  
3. Observar en la interfaz los valores de retorno de posición y velocidad.  
4. Repetir con distintos valores de consigna y monitorear la estabilidad de la conexión.

### Criterios de Aceptación

- La interfaz web es accesible y funcional en la red local.  
- Los valores enviados se reflejan de manera inmediata en el comportamiento del motor.  
- La retroalimentación numérica del motor es legible y estable en la interfaz.  
- No se observan desconexiones o retrasos excesivos durante las pruebas.

Estas pruebas validan el funcionamiento conjunto de los tres motores que forman la pierna.

## Prueba de Movimiento Independiente de Cada Motor

### Propósito

Comprobar que cada motor responda correctamente a comandos de posición sin afectar negativamente a los otros ejes.

### Procedimiento

1. Conectar la ESP32 al bus CAN y energizar la pierna robótica.  
2. Enviar comandos de posición para mover únicamente el Motor 1 mientras los otros están en una posición fija.  
3. Repetir el proceso con el Motor 2 y el Motor 3 de manera individual.  
4. Verificar que no haya interferencia o ruidos en los demás motores durante el movimiento de uno solo.

### Criterios de Aceptación

- Cada motor realiza el movimiento indicado con precisión y sin sobresaltos.  
- Los motores no controlados se mantienen estables en su posición (si así lo define el firmware).  
- La retroalimentación de posición y velocidad para cada motor es coherente con lo esperado.

## Switch de Emergencia y Switches de Cada Motor

### Propósito 
Asegurar que el circuito de emergencia y los interruptores individuales de cada motor puedan detener el sistema en caso de falla.

### Procedimiento 

1. Encender la pierna robótica y comenzar un movimiento (posición o velocidad).  
2. Accionar el botón de emergencia y verificar que todos los motores se detengan inmediatamente.  
3. Repetir accionando únicamente el switch de un motor específico (si la arquitectura eléctrica lo permite).

### Criterios de Aceptación

- Los motores dejan de recibir alimentación o señales de control al presionar el botón de emergencia.  
- Accionar el switch de un motor *solo* detiene o deshabilita ese eje, sin afectar los otros, si así está diseñado el sistema.

## Límites de Rango de Movimiento

### Procedimiento

1. Definir en el firmware los límites máximos y mínimos de movimiento para cada articulación.  
2. Comandar un movimiento que intente sobrepasar dichos límites y verificar que el sistema lo bloquee.  
3. Observar el comportamiento de la pierna al llegar a los topes físicos.

### Criterios de Aceptación

- El firmware detiene o limita el movimiento antes de alcanzar el tope físico.  
- No se producen daños en las piezas impresas o en el cableado.
