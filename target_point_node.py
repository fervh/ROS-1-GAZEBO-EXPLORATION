#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TransformStamped, Point
from sensor_msgs.msg import PointCloud
import tf2_ros
import numpy as np

def point_cloud_callback(msg):
    # Extrae los puntos del mensaje PointCloud
    points = np.array([[p.x, p.y, p.z] for p in msg.points])

    # Calcula el centroide de la nube de puntos
    centroid = np.mean(points, axis=0)

    #Calcula la zona con mayor densidad de puntos que pertenezcan a la nube de puntos
    mediana = np.median(points, axis=0)
    mean = np.mean(points, axis=0)






    # Calcula el punto de la nube de puntos más cercano al centroide
    closest_point = points[np.argmin(np.linalg.norm(points - centroid, axis=1))]
    print("Punto más cercano al centroide: ", closest_point)

    # Calcula el punto mas cercano al centroide que no este en colision con un borde ocupado (usando el topic /borde_ocupado)

    # Publica la transformación del punto más cercano al centroide
    tf_msg = TransformStamped()
    tf_msg.header.stamp = rospy.Time.now()
    tf_msg.header.frame_id = "map"
    tf_msg.child_frame_id = "middle_point"
    tf_msg.transform.translation.x = mediana[0]
    tf_msg.transform.translation.y = mediana[1]
    tf_msg.transform.translation.z = mediana[2]
    tf_msg.transform.rotation.w = 1.0  # No rotation

    tf_broadcaster.sendTransform(tf_msg)

if __name__ == "__main__":
    rospy.init_node("target_point_node")

    # Configura un suscriptor para el PointCloud '/borde_libre'
    rospy.Subscriber("/borde_libre", PointCloud, point_cloud_callback)

    # Inicializa el emisor de TF
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    rospy.spin()
