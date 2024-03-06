# ROS-1-GAZEBO-EXPLORATION
Este es un workspace de ROS donde donde objetivo principal es desarrollar un algoritmo de exploración autónoma utilizando un robot en un entorno desconocido.

### AUTORES:

**- Fernando Vela Hidalgo (GitHub: [github.com/fervh](https://github.com/fervh))**

**- Javier Gómez Eguizábal (GitHub: [github.com/Javierge15](https://github.com/Javierge15))**

**- Alberto Gil Cuadrado (GitHub: [github.com/Alberto-gca](https://github.com/Alberto-gca))**

<p align="center">
  <img src="https://upload.wikimedia.org/wikipedia/en/thumb/5/5e/Gazebo_logo_without_text.svg/480px-Gazebo_logo_without_text.svg.png" width = 20%/>
</p>

Este proyecto es parte de la asignatura "Robo### ENTORNO 4
- ### Mapa
- ### Tiempo de exploracion
- ### Porcentaje de zona explorada


## VIDEOS
ts Móviles" en la Universidad Carlos III de Madrid (UC3M).

----

## CONTENIDO DEL REPOSITORIO

Este repositorio contiene el paquete de ROS 1 Noetic con los nodos y launch files necesarios para la exploración autónoma.

La organización del paquete es la siguiente:


### NODOS:

- La carpeta `src` contiene los siguientes nodos:
  - [exploration_node.py](ros_autonomous_exploration/src/exploration_node.py): Utiliza el paquete move_base para enviar metas de navegación al robot implementado.
  - [porcentaje_borde_node.py](ros_autonomous_exploration/src/porcentaje_borde_node.py): Determina el porcentaje de exploración del mapa y crea listas para almacenar los puntos del borde del mapa y mensajes de nube de puntos para cada tipo de celda.
  - [target_point_node.py](ros_autonomous_exploration/src/target_point_node.py): Procesa el mensaje de nube de puntos del nodo anterior y determina el punto medio hacia donde se dirigirá el robot.
  - [map_save_node.py](ros_autonomous_exploration/src/map_save_node.py): Comprueba el criterio de parada, guarda el mapa y detiene todos los procesos.

### LAUNCH FILE:

- La carpeta `launch` contiene el archivo [moviles.launch](ros_autonomous_exploration/launch/moviles.launch) que se utiliza para iniciar el algoritmo de exploración.

### RVIZ CONFIG:

- Tambien tiene una configuración por defecto de Rviz [default.rviz](ros_autonomous_exploration/rviz/default.rviz)


## PUESTA EN MARCHA DEL ALGORITMO Y ENTORNOS

### Requiere Ubuntu 20.04 con ROS Noetic

#### - En la Terminal 1:
```bash
roslaunch ros_autonomous_exploration moviles.launch
```
(Aquí se puede cambiar el escenario que se quiere lanzar)

#### - En la Terminal 2:
```bash
rosrun ros_autonomous_exploration map_save_node.py
```
(Aquí se puede elegir el nombre del mapa que se va a guardar)

#### - En la Terminal 3:
```bash
rostopic echo /porcentaje_borde
```
(Sirve para tener conocimiento del porcentaje de borde escaneado)


----

## ALGORITMO DE EXPLORACIÓN
Tanto el el algoritmo de exploración como el criterio de parada se encuentra en un sistema diseñado de manera modular, por lo que el código se encuentra dividido en cuatro scripts dentro del paquete de ROS, en los que cada uno realiza una función primordial dentro del algoritmo. Estos scripts son los siguientes:

### exploration_node
[exploration_node.py](ros_autonomous_exploration/src/exploration_node.py)
Utiliza el paquete move_base para enviar metas de navegación al robot implementado.

### porcentaje_borde_node
[porcentaje_borde_node.py](ros_autonomous_exploration/src/porcentaje_borde_node.py)
Determina cual es el porcentaje de exploración del mapa. Crea listas para almacenar los puntos del borde del mapa y mensajes de nube de puntos para cada tipo de celda.

### target_point_node
[target_point_node.py](ros_autonomous_exploration/src/target_point_node.py)
Procesa el mensaje de nube de puntos del nodo anterior y determina el punto medio de esta, hacia donde se dirigirá posteriormente el robot.

### map_save_node
[map_save_node.py](ros_autonomous_exploration/src/map_save_node.py)
Comprueba el criterio de parada, guarda el mapa y detiene todos los procesos. La llamada a este código determina el final de la ejecución del algoritmo.


----

## CRITERIO DE PARADA 

En el momento que el porcentaje del borde en contacto con obstáculos supera el deseado y determinado en el código con respecto al borde en contacto con zona inexplorada se asume el mapa como completado, se guarda y se detienen todos los procesos. Para cada caso se puede definir el porcentaje deseado. 

----

## RESULTADOS OBTENIDOS
### ESCENARIO 1
- ### Mapa
  Capturas del proceso de mapeado
  <p align="center">
    <img src="media/img/map1_generacion.jpg" width = 70%/>
  </p>
  Una vez completado, el mapa se presenta de la siguiente manera:
  <p align="center">
    <img src="media/img/map1_generado.jpg" width = 70%/>
  </p>

- ### Tiempo de exploracion
  El tiempo de exploración del mapa es de 33 segundos. El proceso se puede ver en el siguiente video:
  
  [escenario1.webm](https://github.com/fervh/ROS-1-GAZEBO-EXPLORATION/assets/55854056/d970587c-2e94-408a-9040-90e5184a3983)
  
- ### Porcentaje de zona explorada

  Se alcanza para este mapa un porcentaje de exploración del 95%.

### ESCENARIO 2
- ### Mapa
  Capturas del proceso de mapeado
  <p align="center">
    <img src="media/img/map2_generacion.jpg" width = 70%/>
  </p>
  Una vez completado, el mapa se presenta de la siguiente manera:
  <p align="center">
    <img src="media/img/map2_generado.jpg" width = 70%/>
  </p>

- ### Tiempo de exploracion
  El tiempo de exploración del mapa es de 3 minutos y 50 segundos. El proceso se puede ver en el siguiente video de forma acelerada:

  [escenario2.webm](https://github.com/fervh/ROS-1-GAZEBO-EXPLORATION/assets/55854056/70e7e5ba-62ca-4ea8-b854-515ec065cf7c)

- ### Porcentaje de zona explorada

  Se alcanza para este mapa un porcentaje de exploración del 85%.
  
### ESCENARIO 3
- ### Mapa
Capturas del proceso de mapeado

- ### Tiempo de exploracion
El tiempo de exploración del mapa es de 2 minutos y 55 segundos. El proceso se puede ver en el siguiente video de forma acelerada:

[escenario3.webm](https://github.com/fervh/ROS-1-GAZEBO-EXPLORATION/assets/55854056/8087233f-e98f-4f4a-9c0e-0cacd7027fd0)

- ### Porcentaje de zona explorada

Se alcanza para este mapa un porcentaje de exploración del 80%.

