#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import subprocess

def porcentaje_borde_callback(data):
    if data.data > 80:
        rospy.loginfo("El porcentaje de borde es mayor que 80. Guardando el mapa...")
        subprocess.call(["rosrun", "map_server", "map_saver", "-f", "mi_mapita"])
        # esperate a que se guarde el mapa
        rospy.sleep(5)
        print("Mapa guardado")
        #rosnode kill -a
        subprocess.call(["rosnode", "kill", "-a"])
    else:
        rospy.loginfo("El porcentaje de borde no es mayor que 80.")

if __name__ == '__main__':
    rospy.init_node('map_saver_node')

    rospy.Subscriber('/porcentaje_borde', Float32, porcentaje_borde_callback)

    rospy.spin()
