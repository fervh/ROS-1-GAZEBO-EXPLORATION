#!/usr/bin/env python

# Este nodo se encarga de guardar el mapa cuando el porcentaje de borde es mayor al predefinido

import rospy
from std_msgs.msg import Float32
import subprocess

def porcentaje_borde_callback(data):
    porcentaje = 90
    if data.data > porcentaje:
        rospy.loginfo(" Guardando el mapa...")
        subprocess.call(["rosrun", "map_server", "map_saver", "-f", "escenario_1"])
        # esperate a que se guarde el mapa
        rospy.sleep(5)
        print("Mapa guardado")
        #rosnode kill -a
        subprocess.call(["rosnode", "kill", "-a"])
    else:
        rospy.loginfo("El porcentaje de borde es menor que que el predefinido")

if __name__ == '__main__':
    rospy.init_node('map_saver_node')

    rospy.Subscriber('/porcentaje_borde', Float32, porcentaje_borde_callback)

    rospy.spin()
