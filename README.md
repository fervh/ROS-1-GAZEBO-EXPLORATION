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

----

## ALGORITMO DE EXPLORACION
sudo apt-get install ros-noetic-dwa-local-planner

----

## CRITERIO DE PARADA 
### Código: [porcentaje_borde_node.py](porcentaje_borde_node.py)

El código superior es un nodo de ros diseñado para evaluar el criterio de parada del algoritmo de exploración explicado con anterioridad.

----

## RESULTADOS OBTENIDOS
### ENTORNO 1
- ### Mapa


- ### Tiempo de exploracion
- ### Porcentaje de zona explorada

### ENTORNO 2
- ### Mapa
- ### Tiempo de exploracion
- ### Porcentaje de zona explorada

### ENTORNO 3
- ### Mapa
- ### Tiempo de exploracion
- ### Porcentaje de zona explorada

### ENTORNO 4
- ### Mapa
- ### Tiempo de exploracion
- ### Porcentaje de zona explorada

## [CODIGO DE EXPLORACION COMENTADO](Codigo.py)

## VIDEOS
