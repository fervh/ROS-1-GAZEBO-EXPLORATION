#!/usr/bin/env python

# Este nodo se encarga de publicar los puntos del borde del mapa en los topics "/borde_libre" y "/borde_ocupado"

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from std_msgs.msg import Header, Float32
from sensor_msgs.msg import PointCloud

def map_callback(msg):
    # Obtener dimensiones del mapa
    width = msg.info.width
    height = msg.info.height
    resolution = msg.info.resolution

    # Obtener origen del mapa
    origin_x = msg.info.origin.position.x
    origin_y = msg.info.origin.position.y

    # Obtener los datos del mapa
    map_data = msg.data

    # Crear listas para almacenar los puntos del borde del mapa
    border_points_libre = []  # Para celdas con valor 0 y pegadas a un vecino -1
    border_points_ocupado = []  # Para celdas con valor 100 y pegadas a un vecino -1

    # Recorrer el mapa para encontrar los puntos del borde
    for x in range(width):
        for y in range(height):
            index = x + y * width
            # Si la celda actual es igual a 0, verificamos sus vecinos
            if map_data[index] == 0:
                # Verificar si está pegada a un vecino -1
                if (x > 0 and map_data[index - 1] == -1) or \
                   (x < width - 1 and map_data[index + 1] == -1) or \
                   (y > 0 and map_data[index - width] == -1) or \
                   (y < height - 1 and map_data[index + width] == -1):
                    border_points_libre.append(Point(x * resolution + origin_x, y * resolution + origin_y, 0))
            # Si la celda actual es igual a 100, verificamos sus vecinos
            elif map_data[index] == 100:
                # Verificar si está pegada a un vecino -1
                if (x > 0 and map_data[index - 1] == -1) or \
                   (x < width - 1 and map_data[index + 1] == -1) or \
                   (y > 0 and map_data[index - width] == -1) or \
                   (y < height - 1 and map_data[index + width] == -1):
                    border_points_ocupado.append(Point(x * resolution + origin_x, y * resolution + origin_y, 0))

    # Crear los mensajes PointCloud para cada tipo de celda
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "map"  # Usar el frame "map"
    
    cloud_msg_libre = PointCloud(header=header, points=border_points_libre)
    cloud_msg_ocupado = PointCloud(header=header, points=border_points_ocupado)

    # Publicar los mensajes PointCloud
    borde_libre_pub.publish(cloud_msg_libre)
    borde_ocupado_pub.publish(cloud_msg_ocupado)

    print("Longitud de border_points_ocupado: ", len(border_points_ocupado))
    print("Longitud de border_points_ocupado: ", len(border_points_libre))
    print("Porcentaje borde: ", len(border_points_ocupado)/(len(border_points_libre)+len(border_points_ocupado))*100, "%")

    percentage_msg = Float32()
    percentage_msg.data = len(border_points_ocupado)/(len(border_points_libre)+len(border_points_ocupado))*100
    porcentaje_borde_pub.publish(percentage_msg)




if __name__ == "__main__":
    rospy.init_node("map_scanner_node")

    # Suscribirse al topic "/map"
    rospy.Subscriber("/map", OccupancyGrid, map_callback)

    # Publicar el topic "/borde_libre" para celdas con valor 0 y pegadas a un vecino -1
    borde_libre_pub = rospy.Publisher("/borde_libre", PointCloud, queue_size=1)
    # Publicar el topic "/borde_ocupado" para celdas con valor 100 y pegadas a un vecino -1
    borde_ocupado_pub = rospy.Publisher("/borde_ocupado", PointCloud, queue_size=1)
    # Publicar el topic float "/porcentaje_borde" para el porcentaje de borde
    porcentaje_borde_pub = rospy.Publisher("/porcentaje_borde", Float32, queue_size=1)
    
    rospy.spin()
