#!/usr/bin/env python

# Para elegir el target point, se calcula la mediana de los puntos de la nube de puntos y se elige el punto más cercano a la mediana
import rospy
from geometry_msgs.msg import TransformStamped, Point
from sensor_msgs.msg import PointCloud
import tf2_ros
import numpy as np

def point_cloud_callback(msg):
    # Extrae los puntos del mensaje PointCloud
    points = np.array([[p.x, p.y, p.z] for p in msg.points])

    #Calcula la zona con mayor densidad de puntos que pertenezcan a la nube de puntos
    mediana = np.median(points, axis=0)
    mean = np.mean(points, axis=0)


    # frontera con mayor densidad de puntos
    max_density = np.max(np.linalg.norm(points - mediana, axis=1))

    # punto aleatorio de la zona con mayor densidad de puntos de la frontera max_density
    random_point = points[np.random.choice(np.where(np.linalg.norm(points - mediana, axis=1) < max_density)[0])]

    # punto más cercano al punto aleatorio
    closest_point = points[np.argmin(np.linalg.norm(points - (random_point) / 1, axis=1))]

    print("Punto más cercano al centroide: ", closest_point)

    # Publica la transformación del punto más cercano al centroide
    tf_msg = TransformStamped()
    tf_msg.header.stamp = rospy.Time.now()
    tf_msg.header.frame_id = "map"
    tf_msg.child_frame_id = "middle_point"
    tf_msg.transform.translation.x = closest_point[0]
    tf_msg.transform.translation.y = closest_point[1]
    tf_msg.transform.translation.z = closest_point[2]
    tf_msg.transform.rotation.w = 1.0  # No rotation

    tf_broadcaster.sendTransform(tf_msg)

if __name__ == "__main__":
    rospy.init_node("target_point_node")

    # Configura un suscriptor para el PointCloud '/borde_libre'
    rospy.Subscriber("/borde_libre", PointCloud, point_cloud_callback)

    # Inicializa el emisor de TF
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    rospy.spin()
