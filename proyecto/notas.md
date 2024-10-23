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
