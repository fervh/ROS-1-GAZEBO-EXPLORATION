# ROS-1-GAZEBO-EXPLORATION
Este es un workspace de ROS donde donde objetivo principal es desarrollar un algoritmo de exploración autónoma utilizando un robot en un entorno desconocido.

### AUTORES:

**- Fernando Vela Hidalgo (GitHub: [github.com/fervh](https://github.com/fervh))**

**- Javier Gómez Eguizábal (GitHub: [github.com/Javierge15](https://github.com/Javierge15))**

**- Alberto Gil Cuadrado (GitHub: [github.com/Alberto-gca](https://github.com/Alberto-gca))**

<p align="center">
  <img src="https://upload.wikimedia.org/wikipedia/en/thumb/5/5e/Gazebo_logo_without_text.svg/480px-Gazebo_logo_without_text.svg.png" width = 20%/>
</p>

Este proyecto es parte de la asignatura "Robots Móviles" en la Universidad Carlos III de Madrid (UC3M).

----
## PUESTA EN MARCHA DEL ALGORITMO Y ENTORNOS

### Reqiere Ubuntu 20.04 con ROS Noetic

En la Terminal 1:
```bash
roslaunch ros_autonomous_exploration moviles.launch
```

En la Terminal 2:
```bash
rosrun ros_autonomous_exploration map_save_node.py
```

En la Terminal 3:
```bash
rostopic echo /porcentaje_borde
```


<p align="center">
  <img src="https://github.com/fervh/ROS-1-GAZEBO-EXPLORATION/assets/148269271/1ef31e40-4414-434f-a0bc-c7932722b5ae" width = 100%/>
</p>


Si da algún error prueba a ejecutar el siguiente comando desde terminal:
```bash
sudo apt-get install ros-noetic-dwa-local-planner
```

----

## ALGORITMO DE EXPLORACIÓN
Tanto el el algoritmo de exploración como el criterio de parada se encuentra en un sistema diseñado de manera modular, por lo que el código se encuentra dividido en cuatro scripts en los que cada uno realiza una función primordial dentro del algoritmo. Estos scripts son los siguientes:


### exploration_node
[exploration_node](exploration_node.py)
Utiliza el paquete move_base para enviar metas de navegación al robot implementado.

### porcentaje_borde_node
[porcentaje_borde_node.py](porcentaje_borde_node.py)
Determina cual es el porcentaje de exploración del mapa. Crea listas para almacenar los puntos del borde del mapa y mensajes de nube de puntos para cada tipo de celda.

### target_point_node
[target_point_node](target_point_node.py)
Procesa el mensaje de nube de puntos del nodo anterior y determina el punto medio de esta, hacia donde se dirigirá posteriormente el robot.

### map_save_node
[map_save_node](map_save_node.py)
Comprueba el criterio de parada, guarda el mapa y detiene todos los procesos. La llamada a este código determina el final de la ejecución del algoritmo.


----

## CRITERIO DE PARADA 

En el momento que el porcentaje del borde en contacto con obstáculos supera el deseado y determinado en el código con respecto al borde en contacto con zona inexplorada se asume el mapa como completado, se guarda y se detienen todos los procesos. Para cada caso se puede definir el porcentaje deseado. En este caso se ha determinado un porcentaje mínimo del 80%

----

## RESULTADOS OBTENIDOS
### ENTORNO 1
- ### Mapa
  **Secuencia de exploración**
<p align="center">
  <img src="https://github.com/fervh/ROS-1-GAZEBO-EXPLORATION/assets/148269271/649f8a59-108f-45d1-8778-4841ea99d76d" width = 80%/>
</p>

  **Mapa Final**
<p align="center">
  <img src="https://github.com/fervh/ROS-1-GAZEBO-EXPLORATION/assets/148269271/d56f5687-3190-4adf-b401-ce667c8d96fe" width = 80%/>
</p>

- ### Tiempo de exploracion
  33''
- ### Porcentaje de zona explorada

### ENTORNO 2
- ### Mapa
  **Secuencia de exploración**
<p align="center">
  <img src="https://github.com/fervh/ROS-1-GAZEBO-EXPLORATION/assets/148269271/a3b7125d-db9e-41e7-b0c5-abc60d3dbc4a" width = 60%/>
</p>

  **Mapa Final**
<p align="center">
  <img src="https://github.com/fervh/ROS-1-GAZEBO-EXPLORATION/assets/148269271/494f6ea6-0d4a-424d-b420-37673f70c9e2" width = 60%/>
</p>

- ### Tiempo de exploracion
  3' 50''
- ### Porcentaje de zona explorada

### ENTORNO 3
- ### Mapa
  **Secuencia de exploración**
<p align="center">
  <img src="" width = 60%/>
</p>

  **Mapa Final**

- ### Tiempo de exploracion
- ### Porcentaje de zona explorada

### ENTORNO 4
- ### Mapa
  **Secuencia de exploración**
<p align="center">
  <img src="" width = 20%/>
</p>

  **Mapa Final**

- ### Tiempo de exploracion
- ### Porcentaje de zona explorada


## VIDEOS

[path_follower.webm](https://github.com/fervh/Webots-Simulator/assets/55854056/836a29f1-5e6e-4470-bc68-1dae830199a0)

https://github.com/fervh/ROS-1-GAZEBO-EXPLORATION/blob/main/media/vid/escenario1.mkv
