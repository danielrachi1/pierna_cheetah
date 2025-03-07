# Pata de robot cheetah

 Este proyecto se enfoca en el diagnóstico, mantenimiento y mejora de una pierna robótica inspirada en la familia Cheetah, cuyo desarrollo se orienta a su uso en prácticas de laboratorio dentro del programa de Ingeniería Mecatrónica de la Universidad Nacional de Colombia. El estudio abarcó un análisis integral de los componentes mecánicos, eléctricos y de software del sistema. Durante la fase de diagnóstico se identificaron problemas tales como fracturas en componentes estructurales, limitaciones en el control debido al uso de Arduino, cableado desorganizado y la ausencia de un software operativo adecuado. Para solucionar estas incidencias se rediseñaron las piezas mecánicas, se optimizaron las conexiones eléctricas, se sustituyó el Arduino por una ESP32 y se reescribió el firmware utilizando el framework ESP-IDF. Además, se implementaron perfiles de movimiento y mecanismos de seguridad que permiten suavizar las transiciones y evitar golpes violentos. Paralelamente, se desarrollaron guías de laboratorio y se documentó detalladamente el uso, la arquitectura y las pruebas del sistema, facilitando su integración en el entorno educativo.

## Ejemplos de uso

En el siguiente video, se pueden apreciar las funcionalidades principales del robot:

<iframe width="560" height="315" src="https://www.youtube.com/embed/v7Muz0Q2M44?si=cd-0rgIbi14Tqvrp" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

Este otro video, aunque inicialmente fue grabado para realizar pruebas de funciones de seguridad, también sirve como ejemplo de funcionalidades más avanzadas, como los mecanismos de seguridad:

<iframe width="560" height="315" src="https://www.youtube.com/embed/EpyHmMzFmkk?si=c0lM2PfDSGEZIcur" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## Estructura del repositorio

Este reposito se divide en 4 carpetas:

1. **cad**: En esta carpeta se encuentran los modelos de todas las piezas del robot. Se incluyeron tanto archivos .ipt como .stl. Incluir los archivos .ipt hace más fácil modificar las piezas usando inventor, mientras que los .stl son un formato más estandar, para poder visualizarlos en cualquier programa, o hacer más fácil su proceso de impresión.
2. **documentos**: En esta carpeta se encuentran todos los documentos del proyecto. Estos documentos cubren:
    2.1. El informe del proyecto.
    2.2. Documentación del robot y su implementación actual.
    2.3. Las guías de laboratorio diseñadas para el robot.
3. **esp32_firmware**: Esta carpeto contiene el firmware de la ESP32. Es un proyecto creado usando el `ESP-IDF`.
4. **matlab**: Incluye algunos scripts usados para probar la funcionalidad del sistema. Se deja poco documentado a proposito ya que algunos de estos pueden servir como soluciones a los laboratorios planteados.
