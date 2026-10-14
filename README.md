# polyphemus
## Integrantes
- Ignacio Facello
- Lucia Lazlo
- Micaela Grande
- Natasha Vodanovich

## How To Run
1. Lanzar el agente de micro-ros 

2. Descargar el componente de microros en microros_ws/components o en ${HOME}/esp_components

3. Configurar las variables en `idf.py menuconfig` > Project Configuration > {dmp_publisher_toggle, gyro_publisher_toggle, complementary_filter_alpha} 

4. Buildear el cliente de microros con el namespace correcto
```bash
MICROROS_NAMESPACE="polyphemus" idf.py build flash
```

### visualizacion

rviz
```bash
ros2 launch camera_bringup imu_tf.launch.py
```

Luego agregar los modelos de robot de interes (diy, gyro, dmp)

error
```bash
ros2 launch camera_bringup plot.launch.py layout:=[dmp,gyro]
```

genera los graficos para comparar el error entre las implementaciones
