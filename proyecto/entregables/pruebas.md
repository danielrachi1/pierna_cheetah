# Informe de Pruebas de Funcionamiento

Este documento describe las pruebas que se realizarán para verificar el correcto funcionamiento de la pierna robótica y de los subsistemas involucrados. Estas pruebas abarcan tanto aspectos mecánicos como electrónicos y de software, de modo que se garantice la fiabilidad y seguridad de todo el dispositivo.

---

## 1. Objetivos de las Pruebas

1. **Verificar la integridad de las conexiones eléctricas y electrónicas**: Confirmar que la fuente de alimentación, los motores, la ESP32 y los transceptores CAN estén correctamente conectados y funcionen de manera segura.  
2. **Asegurar el correcto funcionamiento del firmware**: Validar las rutinas de envío y recepción de comandos, así como el procesamiento de datos provenientes de los motores.  
3. **Evaluar la respuesta mecánica y cinemática de cada motor**: Garantizar que los motores respondan de forma estable al control de posición, velocidad y torque.  
4. **Validar la interfaz de usuario**: Comprobar que la página web de control (y/o los controladores físicos) permita manejar la pierna de manera intuitiva y confiable.  
5. **Documentar los resultados**: Recopilar información sobre el rendimiento del sistema y generar pautas para el ajuste de parámetros o mantenimiento futuro.

---

## 2. Pruebas Eléctricas y de Cableado

### 2.1 Continuidad de Circuitos

**Propósito**  
Verificar que no existan cortos, conexiones abiertas o soldaduras defectuosas en la fuente de alimentación, interruptores, motores y transceptores CAN.

**Procedimiento**  
1. Usar un multímetro en modo continuidad para revisar todos los cables de alimentación (24 V y 5 V, si aplica).  
2. Revisar el cableado de datos CAN (bus CANH y CANL) entre la ESP32 y cada motor.  
3. Verificar la conexión de los interruptores de emergencia y de los switches individuales de cada motor.

**Criterios de Aceptación**  
- No debe haber continuidad entre líneas de alimentación y masa (GND) salvo donde corresponda.  
- Las líneas de datos deben mostrar continuidad adecuada al bus CAN (no abiertos ni en corto entre sí).  
- Cada interruptor debe abrir y cerrar el circuito sin falsos contactos.

---

## 3. Pruebas de Comunicación y Firmware

### 3.1 Conexión CAN Básica

**Propósito**  
Asegurar que la ESP32 sea capaz de enviar y recibir mensajes CAN mediante el transceptor configurado (por ejemplo, SN65HVD230).

**Procedimiento**  
1. Cargar en la ESP32 un firmware básico (como el `TWAI Self Test Example` del ESP-IDF o equivalente) que envíe tramas de prueba.  
2. Monitorear los logs enviados por la ESP32 para verificar la transmisión y recepción CAN.

**Criterios de Aceptación**  
- Se detectan sin errores todos los mensajes emitidos por la ESP32 (ya sea con un analizador CAN externo o por medio de logs).  
- La ESP32 recibe correctamente (sin errores) los mensajes del propio test (loopback o self-test).

### 3.3 Prueba del Control y la Interfaz Web

**Propósito**  
Asegurar que la interfaz de usuario, accesible vía WiFi, permita enviar comandos de control a los motores y visualizar su estado en tiempo real.

**Procedimiento**  
1. Ingresar a la página de control alojada en la ESP32 (`http://cheetah.local`) desde un computador o dispositivo móvil.  
2. Enviar un comando de posición al motor 1.  
3. Observar en la interfaz los valores de retorno de posición y velocidad.  
4. Repetir con distintos valores de consigna y monitorear la estabilidad de la conexión.

**Criterios de Aceptación**  
- La interfaz web es accesible y funcional en la red local.  
- Los valores enviados se reflejan de manera inmediata en el comportamiento del motor.  
- La retroalimentación numérica del motor es legible y estable en la interfaz.  
- No se observan desconexiones o retrasos excesivos durante las pruebas.

---

## 4. Pruebas de Cada Grado de Libertad

Estas pruebas validan el funcionamiento conjunto de los tres motores que forman la pierna.

### 4.1 Prueba de Movimiento Independiente de Cada Motor

**Propósito**  
Comprobar que cada motor responda correctamente a comandos de posición sin afectar negativamente a los otros ejes.

**Procedimiento**  
1. Conectar la ESP32 al bus CAN y energizar la pierna robótica.  
2. Enviar comandos de posición para mover únicamente el Motor 1 mientras los otros están en una posición fija.  
3. Repetir el proceso con el Motor 2 y el Motor 3 de manera individual.  
4. Verificar que no haya interferencia o ruidos en los demás motores durante el movimiento de uno solo.

**Criterios de Aceptación**  
- Cada motor realiza el movimiento indicado con precisión y sin sobresaltos.  
- Los motores no controlados se mantienen estables en su posición (si así lo define el firmware).  
- La retroalimentación de posición y velocidad para cada motor es coherente con lo esperado.

---

## 6. Pruebas de Seguridad

### 6.1 Switch de Emergencia y Switches de Cada Motor

**Propósito**  
Asegurar que el circuito de emergencia y los interruptores individuales de cada motor puedan detener el sistema en caso de falla.

**Procedimiento**  
1. Encender la pierna robótica y comenzar un movimiento (posición o velocidad).  
2. Accionar el botón de emergencia y verificar que todos los motores se detengan inmediatamente.  
3. Repetir accionando únicamente el switch de un motor específico (si la arquitectura eléctrica lo permite).

**Criterios de Aceptación**  
- Los motores dejan de recibir alimentación o señales de control al presionar el botón de emergencia.  
- Accionar el switch de un motor *solo* detiene o deshabilita ese eje, sin afectar los otros, si así está diseñado el sistema.

### 6.2 Límites de Rango de Movimiento

**Propósito**  
Evitar daños mecánicos o desconexiones de cables cuando el motor supere un ángulo seguro.

**Procedimiento**  
1. Definir en el firmware los límites máximos y mínimos de movimiento para cada articulación.  
2. Comandar un movimiento que intente sobrepasar dichos límites y verificar que el sistema lo bloquee.  
3. Observar el comportamiento de la pierna al llegar a los topes físicos.

**Criterios de Aceptación**  
- El firmware detiene o limita el movimiento antes de alcanzar el tope físico.  
- No se producen daños en las piezas impresas o en el cableado.

---

## 8. Registro de Resultados y Ajustes Futuros

Después de cada prueba, se registrarán los datos relevantes en un formato de bitácora o en el software de monitorización que se haya implementado. En caso de encontrar problemas o desviaciones, se documentarán las causas y las acciones correctivas. Entre estas podrían incluirse:

- Ajustes en el cableado.  
- Cambios de firmware (correcciones o mejoras en algoritmos de control).  
- Sustitución de piezas mecánicas.  
- Modificaciones en la interfaz de usuario o en la integración con ROS2, si aplica.

---

## 9. Conclusiones

La batería de pruebas descrita en este documento permite evaluar la fiabilidad, precisión y seguridad de la pierna robótica. Siguiendo cada conjunto de pruebas, se podrá garantizar que el dispositivo cumple con los objetivos y resultados esperados. Asimismo, quedarán registradas pautas claras de operación y mantenimiento, lo que facilitará futuras actualizaciones y mejoras del sistema.
