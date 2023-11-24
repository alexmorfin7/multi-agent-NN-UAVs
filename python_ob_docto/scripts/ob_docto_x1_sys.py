#!/usr/bin/env python
# ROS python API
import rospy
import numpy as np
#string header
from std_msgs.msg import String, Header
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import *#Point, PoseStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
#import math for arctan and sqrt function
from math import atan2, sqrt, pi, cos, sin
#import quaternion transformation
from tf.transformations import euler_from_quaternion, quaternion_from_euler
#from numpy import *
# Flight modes class
# Flight modes are activated using ROS services
import MultirotorModule as mrm
# Main function
def main():
    #global archivo
    # initiate node
    rospy.init_node('experiment_sys_z', anonymous=True)

    # flight mode object
    modes = mrm.fcuModes()
#Archivo donde indica el numero de prueba
    last=open('/home/alexmorfin/catkin_ws/src/python_ob_docto/scripts/Pruebas_multi_sys/last.txt','r')
    archivo=last.read()
    archivo=int(archivo)+1
    last=open('/home/alexmorfin/catkin_ws/src/python_ob_docto/scripts/Pruebas_multi_sys/last.txt','w')
    last.write(str(archivo))
    last.close()
	
    #Crear Archivo de datos o de prueba
    f_csv=open('/home/alexmorfin/catkin_ws/src/python_ob_docto/scripts/Pruebas_multi_sys/prueba_%s.txt'%(archivo),'a')
    f_csv.write('time, uz, alt, vel_alt, uthe, roll, uphi, pitch, upsi, yaw'+'\n')

    # controller object
    cnt = mrm.Controller(archivo)

    # ROS loop rate
    rate = rospy.Rate(1/cnt.dt)

    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, cnt.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.poseCb)
    rospy.Subscriber('mavros/local_position/velocity_local', TwistStamped, cnt.velCb)
    #rospy.Subscriber('cmd_vel', Twist, cnt.poseCb)
    # Setpoint publisher    
    at_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
    #cmd_pub = rospy.Publisher('cmd_vel', AttitudeTarget, queue_size=1)
    
    # Make sure the drone is armed
    while not cnt.state.armed:
        modes.setArm()
	rate.sleep()

    # set in takeoff mode and takeoff to default altitude (3 m)
    # modes.setTakeoff()
    # rate.sleep()

    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    k=0
    while k<10:
        at_pub.publish(cnt.at)
        rate.sleep()
        k = k + 1

    # activate OFFBOARD mode
    modes.setOffboardMode()
    
    # ROS main loop
    while not rospy.is_shutdown():
    	cnt.updateSp()
    	at_pub.publish(cnt.at)
	rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
