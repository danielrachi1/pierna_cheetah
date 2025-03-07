# Pata de robot cheetah

 Este proyecto se enfoca en el diagnóstico, mantenimiento y mejora de una pierna robótica inspirada en la familia Cheetah, cuyo desarrollo se orienta a su uso en prácticas de laboratorio dentro del programa de Ingeniería Mecatrónica de la Universidad Nacional de Colombia. El estudio abarcó un análisis integral de los componentes mecánicos, eléctricos y de software del sistema. Durante la fase de diagnóstico se identificaron problemas tales como fracturas en componentes estructurales, limitaciones en el control debido al uso de Arduino, cableado desorganizado y la ausencia de un software operativo adecuado. Para solucionar estas incidencias se rediseñaron las piezas mecánicas, se optimizaron las conexiones eléctricas, se sustituyó el Arduino por una ESP32 y se reescribió el firmware utilizando el framework ESP-IDF. Además, se implementaron perfiles de movimiento y mecanismos de seguridad que permiten suavizar las transiciones y evitar golpes violentos. Paralelamente, se desarrollaron guías de laboratorio y se documentó detalladamente el uso, la arquitectura y las pruebas del sistema, facilitando su integración en el entorno educativo.

## Ejemplos de uso

En [este video](https://www.youtube.com/watch?v=v7Muz0Q2M44), se pueden apreciar las funcionalidades principales del robot.

[Este otro video](https://www.youtube.com/watch?v=EpyHmMzFmkk), aunque inicialmente fue grabado para realizar pruebas de funciones de seguridad, también sirve como ejemplo de funcionalidades más avanzadas, como los mecanismos de seguridad.

## Estructura del repositorio

Este reposito se divide en 4 carpetas:

1. **cad**: En esta carpeta se encuentran los modelos de todas las piezas del robot. Se incluyeron tanto archivos .ipt como .stl. Incluir los archivos .ipt hace más fácil modificar las piezas usando inventor, mientras que los .stl son un formato más estandar, para poder visualizarlos en cualquier programa, o hacer más fácil su proceso de impresión.
2. **documentos**: La documentación del proyecto. Estos documentos cubren: El informe del proyecto, documentación del robot y su implementación actual, y las guías de laboratorio diseñadas para el robot. Además de los archivos PDF, se incluye una subcarpeta con todos los ZIPs de los proyectos latex. Esto se hace con el fin de facilitar la edición de cada documento. No se agrega un ZIP del informe.
3. **esp32_firmware**: Esta carpeta contiene el firmware de la ESP32. Es un proyecto creado usando el `ESP-IDF`.
4. **matlab**: Incluye algunos scripts usados para probar la funcionalidad del sistema. Se deja poco documentado a proposito ya que algunos de estos pueden servir como pistas para solucionar los laboratorios planteados.
