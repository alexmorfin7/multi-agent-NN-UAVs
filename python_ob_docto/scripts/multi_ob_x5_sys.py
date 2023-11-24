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
import uav0Module as uav0
import uav1Module as uav1
import uav2Module as uav2
import uav3Module as uav3
import uav4Module as uav4
import pdb
N = 5
cop=1#0.5
ref_z=6.0
k_f=0.3
lam=40
Gamma=600
kte=0.01
F=0.0
u1=np.zeros((1,1))
u2=np.zeros((1,1))
u3=np.zeros((1,1))
u4=np.zeros((1,1))
# Main function
def main():
    ###############intial values########
    Ad = np.array([[0.,1.,1.,0.],[1.,0.,0.,1.],[1.,0.,0.,1.],[0.,1.,1.,0.]])
    D = np.array([[2.,0.,0.,0.],[0.,2.,0.,0.],[0.,0.,2.,0.],[0.,0.,0.,2.]])
    A0 = np.array([[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]])
    K = np.array([[-0.083,-0.343]]) #0.17,0.69
    
    # initiate node
    rospy.init_node('multi_test_sys_z', anonymous=True)
#Archivo donde indica el numero de prueba
    last=open('/home/alexmorfin/catkin_ws/src/python_ob_docto/scripts/Pruebas_multi_sys/last.txt','r')
    archivo=last.read()
    archivo=int(archivo)+1
    last=open('/home/alexmorfin/catkin_ws/src/python_ob_docto/scripts/Pruebas_multi_sys/last.txt','w')
    last.write(str(archivo))
    last.close()
    d_csv={}
    #e_i={}
    #Crear Archivo de datos o de prueba
    for i in range(N):	
	d_csv['f_csv{}'.format(i)]=open('/home/alexmorfin/catkin_ws/src/python_ob_docto/scripts/Pruebas_multi_sys/prueba_%s_uav%s.txt'%(archivo,i),'a')
        d_csv['f_csv{}'.format(i)].write('time, uz, alt, vel_alt, uthe, roll, uphi, pitch, upsi, yaw, F'+'\n')
	d_csv['f_csv{}'.format(i)].close()
##UAV leader#########################
    # flight mode object
    modes_0 = uav0.fcuModes()
    # controller object
    cnt_0 = uav0.Controller(archivo)

    # ROS loop rate
    rate = rospy.Rate(1/cnt_0.dt)

    # Subscribe to drone state
    rospy.Subscriber('uav0/mavros/state', State, cnt_0.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('uav0/mavros/local_position/pose', PoseStamped, cnt_0.poseCb)
    rospy.Subscriber('uav0/mavros/local_position/velocity_local', TwistStamped, cnt_0.velCb)
    #rospy.Subscriber('cmd_vel', Twist, cnt.poseCb)
    # Setpoint publisher    
    at_pub0 = rospy.Publisher('uav0/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
    #cmd_pub = rospy.Publisher('cmd_vel', AttitudeTarget, queue_size=1)
############UAV 1####################################
    # flight mode object
    modes_1 = uav1.fcuModes()
    # controller object
    cnt_1 = uav1.Controller(archivo)

    # Subscribe to drone state
    rospy.Subscriber('uav1/mavros/state', State, cnt_1.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('uav1/mavros/local_position/pose', PoseStamped, cnt_1.poseCb)
    rospy.Subscriber('uav1/mavros/local_position/velocity_local', TwistStamped, cnt_1.velCb)
    #rospy.Subscriber('cmd_vel', Twist, cnt.poseCb)
    # Setpoint publisher    
    at_pub1 = rospy.Publisher('uav1/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
    #cmd_pub = rospy.Publisher('cmd_vel', AttitudeTarget, queue_size=1)
############UAV 2####################################
    # flight mode object
    modes_2 = uav2.fcuModes()
    # controller object
    cnt_2 = uav2.Controller(archivo)

    # Subscribe to drone state
    rospy.Subscriber('uav2/mavros/state', State, cnt_2.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('uav2/mavros/local_position/pose', PoseStamped, cnt_2.poseCb)
    rospy.Subscriber('uav2/mavros/local_position/velocity_local', TwistStamped, cnt_2.velCb)
    #rospy.Subscriber('cmd_vel', Twist, cnt.poseCb)
    # Setpoint publisher    
    at_pub2 = rospy.Publisher('uav2/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
    #cmd_pub = rospy.Publisher('cmd_vel', AttitudeTarget, queue_size=1)
############UAV 3####################################
    # flight mode object
    modes_3 = uav3.fcuModes()
    # controller object
    cnt_3 = uav3.Controller(archivo)

    # Subscribe to drone state
    rospy.Subscriber('uav3/mavros/state', State, cnt_3.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('uav3/mavros/local_position/pose', PoseStamped, cnt_3.poseCb)
    rospy.Subscriber('uav3/mavros/local_position/velocity_local', TwistStamped, cnt_3.velCb)
    #rospy.Subscriber('cmd_vel', Twist, cnt.poseCb)
    # Setpoint publisher    
    at_pub3 = rospy.Publisher('uav3/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
    #cmd_pub = rospy.Publisher('cmd_vel', AttitudeTarget, queue_size=1)
############UAV 4####################################
    # flight mode object
    modes_4 = uav4.fcuModes()
    # controller object
    cnt_4 = uav4.Controller(archivo)

    # Subscribe to drone state
    rospy.Subscriber('uav4/mavros/state', State, cnt_4.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('uav4/mavros/local_position/pose', PoseStamped, cnt_4.poseCb)
    rospy.Subscriber('uav4/mavros/local_position/velocity_local', TwistStamped, cnt_4.velCb)
    #rospy.Subscriber('cmd_vel', Twist, cnt.poseCb)
    # Setpoint publisher    
    at_pub4 = rospy.Publisher('uav4/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
    #cmd_pub = rospy.Publisher('cmd_vel', AttitudeTarget, queue_size=1)    
    # Make sure the drone is armed
    while not (cnt_0.state.armed and cnt_1.state.armed and cnt_2.state.armed and cnt_3.state.armed and cnt_4.state.armed):
        modes_0.setArm()
	modes_1.setArm()
	modes_2.setArm()
	modes_4.setArm()
	modes_3.setArm()
	

	rate.sleep()

    # set in takeoff mode and takeoff to default altitude (3 m)
    # modes.setTakeoff()
    # rate.sleep()

    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    k=0
    while k<10:
        at_pub0.publish(cnt_0.at)
	at_pub1.publish(cnt_1.at)
	at_pub2.publish(cnt_2.at)
	at_pub3.publish(cnt_3.at)
	at_pub4.publish(cnt_4.at)
        rate.sleep()
        k = k + 1

    # activate OFFBOARD mode
    modes_0.setOffboardMode()
    modes_1.setOffboardMode()
    modes_2.setOffboardMode()
    modes_3.setOffboardMode()	
    modes_4.setOffboardMode()
    		
    u1=0.
    u2=0.
    u3=0.
    u4=0.
    
    # ROS main loop
    while not rospy.is_shutdown():
    	cnt_0.updateSp(ref_z,kp=0.3618,kd=0.47)#0.036818,0.12893#0.35,0.45
	
#########Calculos#########################
###############estados###########
	#for i in range(1,N+1)
	x0=np.array([[cnt_0.pos_uav.z],[cnt_0.vel_uav.z]])
	x1=np.array([[cnt_1.pos_uav.z],[cnt_1.vel_uav.z]])
	x2=np.array([[cnt_2.pos_uav.z],[cnt_2.vel_uav.z]])
	x3=np.array([[cnt_3.pos_uav.z],[cnt_3.vel_uav.z]])
	x4=np.array([[cnt_4.pos_uav.z],[cnt_4.vel_uav.z]])
	#e_i={}
    #Crear errores
	#for p in range(1,N+1):
        #	for m in range(1,N+1):
    	#		e_i['e_{}'.format(p)]=Ad[p,m]*('x{}'%(p)-'x{}'%(m))+A0[p,p]*('x{}'%(p)-x0)
        	
	e_1=Ad[0,1]*(x1-x2)+Ad[0,2]*(x1-x3)+Ad[0,3]*(x1-x4)+A0[0,0]*(x1-x0)
	e_2=Ad[1,0]*(x2-x1)+Ad[1,2]*(x2-x3)+Ad[1,3]*(x2-x4)+A0[1,1]*(x2-x0)
	e_3=Ad[2,0]*(x3-x1)+Ad[2,1]*(x3-x2)+Ad[2,3]*(x3-x4)+A0[2,2]*(x3-x0)
	e_4=Ad[3,0]*(x4-x1)+Ad[3,1]*(x4-x2)+Ad[3,2]*(x4-x3)+A0[3,3]*(x4-x0)
	#print(e_1)
	#print(e_2)
	#print(e_3)
	#print(e_4)
	#pdb.set_trace()
	
	F1=cnt_1.brf_z(e_1,u1,k_f,lam,Gamma,kte)
	u1=cop*np.dot(K,e_1)-F1

	F2=cnt_2.brf_z(e_2,u2,k_f,lam,Gamma,kte)
	u2=cop*np.dot(K,e_2)-F2

	F3=cnt_3.brf_z(e_3,u3,k_f,lam,Gamma,kte)
	u3=cop*np.dot(K,e_3)-F3

	F4=cnt_4.brf_z(e_4,u4,k_f,lam,Gamma,kte)
	u4=cop*np.dot(K,e_4)-F4
	#print(F1)
	#print(F2)
	#print(F3)
	print(F4)
	#pdb.set_trace()	
	cnt_1.updateSp(u1)
	cnt_2.updateSp(u2)
	cnt_3.updateSp(u3)
	cnt_4.updateSp(u4)

	at_pub0.publish(cnt_0.at)
    	at_pub1.publish(cnt_1.at)
	at_pub2.publish(cnt_2.at)
	at_pub3.publish(cnt_3.at)
	at_pub4.publish(cnt_4.at)
	
	rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
