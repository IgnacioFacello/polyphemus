Proyecto de ROS2 que lee los valores publicados por la ESP32

`src/` contiene los paquetes

`bot_bringup/` es el paquete de 'bringup'. Contiene scripts para lanzar todo el proyecto de una.
Correr `ros2 launch bot_bringup bot_app.launch.yaml`

`control_service/` contiene el nodo de ROS2 escrito en python. Este nodo escucha los valores publicados en el topico `/sensor_data` y los muestra por pantalla de manera gráfica. 
Investigar ROS2, topicos, mensajes y paradigma pub/sub
