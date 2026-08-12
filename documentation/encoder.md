# `encoder.c`
Libreria de manejo de encoders rotatorios.

Trabaja con encoders de doble output (A y B). Al medir la cantidad y el orden de los pulsos podemos calcular el **desplazamiento** y la **dirección**.

Utiliza las unidades 'pulse counters' de la ESP32 para la deteccion y conteo de los pulsos del encoder. Se utilizan estas unidades para mejorar la precision del conteo.

## Uso
`encoder_init()` - Inicializa un nuevo encoder. Recibe los pines en los que se va a conectar el mismo, configura el hardware e inicia el conteo.
`encoder_get_count()`	- Devuelve el conteo actual, positivo o negativo segun la dirección.`encoder_clear_count()` - Reinicia el conteo a 0
`encoder_deinit()` - Desactiva los encoders y libera memoria
