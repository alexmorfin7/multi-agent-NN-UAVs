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
import uav0Modulexy as uav0
import uav1Modulexy as uav1
import pdb
N = 2
cop=1.0#.5
ref_z=4.0
ref_x=0.0
ref_y=0.0
kp_z=1.55#0.35
kd_z=0.75#0.45
kp=0.15
kd=0.2

k_fz=0.3
lamz=40
Gammaz=900
ktez=0.03

k_f=0.3
lam=30
Gamma=100
kte=0.0007
u1=np.zeros((1,1))
# Main function
def main():
    ###############intial values########
    Ad = np.array([[1.]])
    D = np.array([[1.]])
    A0 = np.array([[1.]])
    K_z = np.array([[-1.55,-0.75]])#-0.975,-0.293 
    K = np.array([[-0.15,-0.2]])#0.020,0.10
    x_init = np.array([[3]])
    y_init = np.array([[3]])
    # initiate node
    rospy.init_node('multi_test_sys_xy', anonymous=True)
#Archivo donde indica el numero de prueba
    last=open('/home/alexmorfin/catkin_ws/src/python_ob_docto/scripts/Pruebas_multi_sys_xy/last.txt','r')
    archivo=last.read()
    archivo=int(archivo)+1
    last=open('/home/alexmorfin/catkin_ws/src/python_ob_docto/scripts/Pruebas_multi_sys_xy/last.txt','w')
    last.write(str(archivo))
    last.close()
    d_csv={}
    e={}
    #Crear Archivo de datos o de prueba
    for i in range(N):	
	d_csv['f_csv{}'.format(i)]=open('/home/alexmorfin/catkin_ws/src/python_ob_docto/scripts/Pruebas_multi_sys_xy/prueba_%s_uav%s.txt'%(archivo,i),'a')
        d_csv['f_csv{}'.format(i)].write('time, uz, alt, vel_alt, uthe, roll, uphi, pitch, upsi, yaw, ux, pos_x, vel_x, uy, pos_y, vel_y, Fz, Fx, Fy'+'\n')
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
    sp_pub0 = rospy.Publisher('uav0/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
    #cmd_pub = rospy.Publisher('cmd_vel', AttitudeTarget, queue_size=1)
############UAV 1####################################
    # flight mode object
    modes_1 = uav1.fcuModes()
    # controller object
    cnt_1 = uav1.Controller(archivo,x_init[0,0],y_init[0,0])

    # Subscribe to drone state
    rospy.Subscriber('uav1/mavros/state', State, cnt_1.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('uav1/mavros/local_position/pose', PoseStamped, cnt_1.poseCb)
    rospy.Subscriber('uav1/mavros/local_position/velocity_local', TwistStamped, cnt_1.velCb)
    #rospy.Subscriber('cmd_vel', Twist, cnt.poseCb)
    # Setpoint publisher    
    at_pub1 = rospy.Publisher('uav1/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
    sp_pub1 = rospy.Publisher('uav1/mavros/setpoint_raw/local', PositionTarget, queue_size=1)	
    #cmd_pub = rospy.Publisher('cmd_vel', AttitudeTarget, queue_size=1)
    
    # Make sure the drone is armed
    while not (cnt_0.state.armed and cnt_1.state.armed):
        modes_0.setArm()
	modes_1.setArm()
	rate.sleep()

    # set in takeoff mode and takeoff to default altitude (3 m)
    # modes.setTakeoff()
    # rate.sleep()

    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    k=0
    while k<10:
        at_pub0.publish(cnt_0.at)
	at_pub1.publish(cnt_1.at)
        rate.sleep()
        k = k + 1

    # activate OFFBOARD mode
    modes_0.setOffboardMode()
    modes_1.setOffboardMode()

    u1=0.0
    ux1=0.0
    uy1=0.0
    time=0.0
    dt=0.05
    #F=0.0
    # ROS main loop
    while not rospy.is_shutdown():
	if (time<=5.0):
		sp_pub0.publish(cnt_0.sp)
		sp_pub1.publish(cnt_1.sp)	
	elif (time>5.0 and time<=20.0):

		cnt_0.updateSp(ref_z,kp_z,kd_z,ref_x,ref_y,kp,kd)

	#########Calculos#########################
	###############estados###########
		#for i in range(1,N+1)
		z0=np.array([[cnt_0.pos_uav.z],[cnt_0.vel_uav.z]])
		z1=np.array([[cnt_1.pos_uav.z-(0.0)],[cnt_1.vel_uav.z]])
		e_1=A0[0,0]*(z1-z0)
		x0=np.array([[cnt_0.pos_uav.x],[cnt_0.vel_uav.x]])
		x1=np.array([[cnt_1.pos_uav.x-(2.)],[cnt_1.vel_uav.x]])
		e_x1=A0[0,0]*(x1-x0)
		y0=np.array([[cnt_0.pos_uav.y],[cnt_0.vel_uav.y]])
		y1=np.array([[cnt_1.pos_uav.y-(2.)],[cnt_1.vel_uav.y]])
		e_y1=A0[0,0]*(y1-y0)
			
		#print(e_1)
		#pdb.set_trace()
		#u1=float(cop*np.dot(K,e_1))-F1
		#F1=cnt_1.brf_z(e_1,u1,k_f,lam,Gamma,kte)
		#print(e_1)
		F=cnt_1.brf_z(e_1,u1,k_fz,lamz,Gammaz,ktez)
		Fx1, Fy1 = cnt_1.brf(e_x1,e_y1,ux1,uy1,k_f,lam,Gamma,kte)
		u1=cop*np.dot(K_z,e_1)-F
		ux1=cop*np.dot(K,e_x1)-Fx1
		uy1=cop*np.dot(K,e_y1)-Fy1
		print(Fx1)
		#pdb.set_trace()	
		cnt_1.updateSp(u1,ux1,uy1)
		at_pub0.publish(cnt_0.at)
	    	at_pub1.publish(cnt_1.at)
	else:
		modes_0.setAutoLandMode()
		modes_1.setAutoLandMode()
	time+=dt
	print(time)		
	rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
