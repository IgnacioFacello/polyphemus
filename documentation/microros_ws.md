Proyecto de idf con microros
`main.c` contiene el codigo que va a ser ejecutado por el microcontrolador (ESP32 en este caso)
`main()` solo crea una tarea para microros (investigar FreeRTOS)
La primera parte de la función `micro_ros_task()` configura la tarea y se conecta mediante wifi al agente de microros que corre en alguna computadora de la red (investigar MicroROS)
En la segunda parte, inicializa los handles para un `publisher` (pub/sub) y un `timer`. El segundo obtiene el valor actual de los encoders mediante la libreria `encoder.h` y se los pasa al `publisher` que los publica en un canal de ROS2 para ser leido por otros nodos.

`encoder.c/.h` es la libreria que maneja los encoders. Utiliza pulse counters para mantener un conteo correcto de la posición actual. Se configura un pulse counter con dos canales, A y B, los cuales se configuran de la siguiente manera:

```C
pcnt_channel_set_edge_action(
  dev->chan_a,
  PCNT_CHANNEL_EDGE_ACTION_DECREASE,
  PCNT_CHANNEL_EDGE_ACTION_INCREASE)
  
pcnt_channel_set_level_action(
  dev->chan_a,
  PCNT_CHANNEL_LEVEL_ACTION_KEEP,
  PCNT_CHANNEL_LEVEL_ACTION_INVERSE)
  
pcnt_channel_set_edge_action(
  dev->chan_b,
  PCNT_CHANNEL_EDGE_ACTION_INCREASE,
  PCNT_CHANNEL_EDGE_ACTION_DECREASE)
  
pcnt_channel_set_level_action(
  dev->chan_b,
  PCNT_CHANNEL_LEVEL_ACTION_KEEP,
  PCNT_CHANNEL_LEVEL_ACTION_INVERSE)
```
