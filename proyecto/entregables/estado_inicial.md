# Análisis del Estado Inicial

## Inspección Física y Funcional

### Inspección Visual

- **Estructura**: Los perfiles de aluminio se encuentran estables y en buen estado.
- **Motor 1**: Se mueve con facilidad en un rango aproximado de 180 grados. Sin embargo, el cableado y la estructura limitan su movimiento.
- **Motor 2**: Se desplaza con facilidad en un rango aproximado de 270 grados, con limitación únicamente impuesta por la estructura.
- **Motor 3** (con poleas): Presenta dificultad para moverse, aunque con suficiente fuerza es posible moverlo paso a paso.
- **Soporte del motor 1**: Se encuentra en buen estado.
- **Soporte del motor 2**: La impresión es de la mejor calidad, pero no parece tener signos de daño.
- **Soporte del motor 3**: Aunque es una impresión de baja calidad, no presenta fracturas visibles.
- **Cadera**: Se observa una fractura considerable en la pieza impresa.
- **Muslo**: Falta una parte de la impresión. Al parecer, sufrió una fractura previa que fue reparada con pegamento (super bonder).
- **Rodilla**: No se observan daños.
- **Pierna**: Se encuentra en buen estado, aunque uno de los tornillos debería ser más corto.
- **Cableado de los motores**: Algunos pines se desconectan al realizar ciertos movimientos. El cableado de alimentación no está bien aislado y puede generar cortos. El cableado de datos no parece ser peligroso, pero está algo desordenado.
- **Botón de emergencia**: Se percibe una pieza suelta en su interior, pero el botón parece funcionar correctamente.
- **Fuente de alimentación**: En buen estado.
- **Switches**: Están bien conectados y no muestran daños visibles en los cables.

Se tomaron fotografías del estado visual actual. Estas imágenes se encuentran en el repositorio, en la carpeta `proyecto/imagenes/`.

### Funcionalidad Básica

Aunque se propuso hacer un diagnóstico de la funcionalidad actual, esto no fue posible debido al estado en el que se encontraron las conexiones del robot. Para más información sobre la situación de las conexiones, ver el diagnóstico detallado.

## Diagnóstico Detallado

### Integridad de los Circuitos Electrónicos

Durante la inspección del sistema, se encontró que:

- El único cable conectado era el que iba de la fuente de poder a la pared y los cables de la fuente de poder a los motores.
- Los módulos CAN, necesarios para la comunicación entre los distintos componentes del sistema, estaban completamente desconectados y almacenados en una bolsa aparte del resto del sistema.
- La Raspberry Pi estaba guardada en su caja, desconectada del sistema.
- No se encontró el Arduino que controlaba el sistema en el proyecto original. Es probable que este dispositivo perteneciera a uno de los miembros del equipo y haya sido retirado una vez entregado el proyecto.

Durante la inspección del sistema, utilicé un multímetro en modo de continuidad para verificar la conexión entre la fuente de alimentación, los switches y los motores. Se comprobó que todo parecía estar correctamente conectado. Sin embargo, inicialmente el circuito del motor 3 no registraba continuidad, pero esto parecía deberse a una mala conexión en uno de los cables del switch.

Dadas estas circunstancias, intentar encender el sistema o probar cualquier funcionalidad básica hubiera sido prematuro sin antes asegurar que los módulos esenciales estuvieran correctamente conectados y el control del sistema estuviera restablecido.

### Análisis del Software Actual

En el informe del proyecto PAI, se menciona que el código del sistema debería encontrarse en el repositorio de GitHub [https://github.com/jolayam/proyecto_PAI](https://github.com/jolayam/proyecto_PAI). Sin embargo, tras revisar el repositorio, no se encontró el código correspondiente. Al revisar la memoria SD de la Raspberry Pi, se recuperaron algunos archivos, pero noté que faltaban otros importantes. Por ejemplo, el archivo `CAN_COM.cpp` que se menciona en el informe no estaba presente en la información descargada de la tarjeta SD, lo que sugiere que parte del código necesario para la comunicación CAN se ha extraviado. Aunque algunos archivos esenciales están disponibles, es probable que el sistema no funcione correctamente sin este archivo, o al menos requiera una actualización considerable. Dado que la versión del software parece desactualizada, se recomienda realizar una reescritura parcial o total del código, especialmente considerando la posible implementación de nuevas plataformas, como la ESP32.
