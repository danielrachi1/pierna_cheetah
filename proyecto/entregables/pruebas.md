 # Informe de Pruebas de Funcionamiento

Este documento describe las pruebas que se realizarán para verificar el correcto funcionamiento de la pierna robótica y de los subsistemas involucrados. Estas pruebas abarcan tanto aspectos mecánicos como electrónicos y de software, de modo que se garantice la fiabilidad y seguridad de todo el dispositivo.

## Prueba del Control y la Interfaz Web

### Propósito

Asegurar que la interfaz de usuario, accesible vía WiFi, permita enviar comandos de control a los motores y visualizar su estado en tiempo real.

### Procedimiento

1. Ingresar a la página de control alojada en la ESP32 (`http://cheetah.local`) desde un computador o dispositivo móvil.  
2. Enviar un comando de posición al motor 1.  
3. Observar en los logs los valores de retorno de posición y velocidad.  
4. Repetir con distintos valores y monitorear la estabilidad de la conexión.

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
3. repetir 2 y 3 usando la api en vez de la UI.

### Criterios de Aceptación

- Cada motor realiza el movimiento indicado con precisión y sin sobresaltos.  
- Los motores no controlados se mantienen estables en su posición (si así lo define el firmware).  
- La retroalimentación de posición y velocidad para cada motor es coherente con lo esperado.

## Switch de Emergencia y Switches de Cada Motor

### Propósito 
Asegurar que el boton de emergencia, el relé y los interruptores individuales de cada motor puedan apagar los motores.

### Procedimiento 

1. Encender la pierna y comenzar un movimiento (posición o velocidad).  
2. Accionar el botón de emergencia y verificar que todos los motores se detengan inmediatamente.  
3. repetir desconectando la esp32 (que da una señal de activación al rele siempre que esté encendida)
4. Repetir accionando únicamente el switch de un motor específico.

### Criterios de Aceptación

- Los motores dejan de recibir alimentación al presionar el botón de emergencia y al desconectar la esp32.  
- Accionar el switch de un motor *solo* detiene o deshabilita ese eje, sin afectar los otros.

