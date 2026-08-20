# polyphemus
## Integrantes
Ignacio Facello
Lucia Lazlo
Micaela Grande
Natasha ???

## How To Run
1. Lanzar el agente de micro-ros 
2. Lanzar el nodo graficador
```bash
ros2 launch bot_bringup bot_app.launch.xml
```

3. Descargar el componente de microros en microros_ws/components o en ${HOME}/esp_componetns

3. Buildear el cliente de microros con el namespace correcto
```bash
MICROROS_NAMESPACE="ifacello" idf.py build 
```

4. Flashear el microcontrolador
```bash
idf.py flash
```
