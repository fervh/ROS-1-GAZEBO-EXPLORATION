#!/usr/bin/env python

# Este nodo se encarga de navegar hacia el punto medio del mapa

# redacta en un comentario una explicación del algoritmo de exploración seleccionado
# El algoritmo de exploración seleccionado es el de navegar hacia el punto medio del mapa. Este algoritmo se encarga de navegar hacia el punto medio del mapa, el cual es publicado por el nodo target_point_node. El punto medio del mapa es el punto más cercano al centroide de la nube de puntos del borde del mapa. El nodo exploration_node se suscribe al topic /middle_point para recibir el punto medio del mapa y navegar hacia él.


import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf2_ros
from geometry_msgs.msg import TransformStamped
from random import randint

def middle_point_callback(tf_msg):
    print('Middle point received')
    select_and_publish_goal(tf_msg)

def select_and_publish_goal(tf_msg):
    print('Navigating to the middle point...')
    goal_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    goal_client.wait_for_server()
    
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = tf_msg.transform.translation.x
    goal.target_pose.pose.position.y = tf_msg.transform.translation.y
    goal.target_pose.pose.orientation.w = 1.0

    goal_client.send_goal(goal)

if __name__ == '__main__':
    try:
        rospy.init_node('exploration', anonymous=True)
        
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        
        rospy.sleep(1)  # Esperar 1 segundo para que el buffer se llene
        
        while not rospy.is_shutdown():
            try:
                tf_msg = tf_buffer.lookup_transform("map", "middle_point", rospy.Time())
                select_and_publish_goal(tf_msg)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn("Failed to get middle point. Trying again...")
            
            rospy.sleep(2) # Esperar 2 segundos antes de intentar de nuevo

    except rospy.ROSInterruptException:
        rospy.loginfo("Exploration finished.")
