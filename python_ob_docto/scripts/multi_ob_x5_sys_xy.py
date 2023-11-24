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
import uav2Modulexy as uav2
import uav3Modulexy as uav3
import uav4Modulexy as uav4
	
#import pdb
N = 5
cop=1.0#.5
ref_z=5.0
ref_x=1.5
ref_y=1.5
des_x=1.5
des_y=1.5
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
Gamma=60
kte=0.03
#u1=np.zeros((1,1))
# Main function
def main():
    ###############intial values########
    Ad = np.array([[0.,1.,1.,1.],[1.,0.,1.,1.],[1.,1.,0.,1.],[1.,1.,1.,0.]])
    D = np.array([[3.,0.,0.,0.],[0.,3.,0.,0.],[0.,0.,3.,0.],[0.,0.,0.,3.]])
    A0 = np.array([[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]])
    K_z = np.array([[-1.55,-0.75]])#-0.975,-0.293 
    K = np.array([[-0.015,-0.02]])#0.020,0.10
    x_init = np.array([[3.,0.,3.,0.]])
    y_init = np.array([[0.,3.,3.,0.]])
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
############UAV 2####################################
    # flight mode object
    modes_2 = uav2.fcuModes()
    # controller object
    cnt_2 = uav2.Controller(archivo,x_init[0,1],y_init[0,1])

    # Subscribe to drone state
    rospy.Subscriber('uav2/mavros/state', State, cnt_2.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('uav2/mavros/local_position/pose', PoseStamped, cnt_2.poseCb)
    rospy.Subscriber('uav2/mavros/local_position/velocity_local', TwistStamped, cnt_2.velCb)
    #rospy.Subscriber('cmd_vel', Twist, cnt.poseCb)
    # Setpoint publisher    
    at_pub2 = rospy.Publisher('uav2/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
    sp_pub2 = rospy.Publisher('uav2/mavros/setpoint_raw/local', PositionTarget, queue_size=1)	
    #cmd_pub = rospy.Publisher('cmd_vel', AttitudeTarget, queue_size=1) 
############UAV 3####################################
    # flight mode object
    modes_3 = uav3.fcuModes()
    # controller object
    cnt_3 = uav3.Controller(archivo,x_init[0,2],y_init[0,2])

    # Subscribe to drone state
    rospy.Subscriber('uav3/mavros/state', State, cnt_3.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('uav3/mavros/local_position/pose', PoseStamped, cnt_3.poseCb)
    rospy.Subscriber('uav3/mavros/local_position/velocity_local', TwistStamped, cnt_3.velCb)
    #rospy.Subscriber('cmd_vel', Twist, cnt.poseCb)
    # Setpoint publisher    
    at_pub3 = rospy.Publisher('uav3/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
    sp_pub3 = rospy.Publisher('uav3/mavros/setpoint_raw/local', PositionTarget, queue_size=1)	
    #cmd_pub = rospy.Publisher('cmd_vel', AttitudeTarget, queue_size=1)   
############UAV 4####################################
    # flight mode object
    modes_4 = uav4.fcuModes()
    # controller object
    cnt_4 = uav4.Controller(archivo,x_init[0,3],y_init[0,3])

    # Subscribe to drone state
    rospy.Subscriber('uav4/mavros/state', State, cnt_4.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('uav4/mavros/local_position/pose', PoseStamped, cnt_4.poseCb)
    rospy.Subscriber('uav4/mavros/local_position/velocity_local', TwistStamped, cnt_4.velCb)
    #rospy.Subscriber('cmd_vel', Twist, cnt.poseCb)
    # Setpoint publisher    
    at_pub4 = rospy.Publisher('uav4/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
    sp_pub4 = rospy.Publisher('uav4/mavros/setpoint_raw/local', PositionTarget, queue_size=1)	
    #cmd_pub = rospy.Publisher('cmd_vel', AttitudeTarget, queue_size=1)   

    # Make sure the drone is armed
    while not (cnt_0.state.armed and cnt_1.state.armed and cnt_2.state.armed and cnt_3.state.armed and cnt_4.state.armed):
        modes_0.setArm()
	modes_1.setArm()
	modes_2.setArm()
	modes_3.setArm()
	modes_4.setArm()
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

    u1=0.0
    ux1=0.0
    uy1=0.0
    u2=0.0
    ux2=0.0
    uy2=0.0
    u3=0.0
    ux3=0.0
    uy3=0.0		
    u4=0.0
    ux4=0.0
    uy4=0.0
    time=0.0
    dt=0.05
    off_x=1.5
    off_y=1.5
    #F=0.0
    # ROS main loop
    while not rospy.is_shutdown():
	if (time<=5.0):
		sp_pub0.publish(cnt_0.sp)
		sp_pub1.publish(cnt_1.sp)
		sp_pub2.publish(cnt_2.sp)
		sp_pub3.publish(cnt_3.sp)
		sp_pub4.publish(cnt_4.sp)	
	elif (time>5.0 and time<=45.0):
		ref_x=(-3*cos((time-5.)*(3.1416*8/180))+3.)+off_x
		ref_y=(-3*sin((time-5.)*(3.1416*8/180)))+off_y
		#print(ref_x,ref_y)
		cnt_0.updateSp(ref_z,kp_z,kd_z,ref_x,ref_y,kp,kd)

	#########Calculos#########################
	###############estados###########
		z0=np.array([[cnt_0.pos_uav.z],[cnt_0.vel_uav.z]])
		z1=np.array([[cnt_1.pos_uav.z],[cnt_1.vel_uav.z]])
		z2=np.array([[cnt_2.pos_uav.z],[cnt_2.vel_uav.z]])
		z3=np.array([[cnt_3.pos_uav.z],[cnt_3.vel_uav.z]])
		z4=np.array([[cnt_4.pos_uav.z],[cnt_4.vel_uav.z]])
		dz1=np.array([[0.],[0.]])
		dz2=np.array([[0.],[0.]])
		dz3=np.array([[0.],[0.]])
		dz4=np.array([[0.],[0.]])
		e_1=Ad[0,1]*(z1-z2-(dz1-dz2))+Ad[0,2]*(z1-z3-(dz1-dz3))+Ad[0,3]*(z1-z4-(dz1-dz4))+A0[0,0]*(z1-z0-dz1)
		e_2=Ad[1,0]*(z2-z1-(dz2-dz1))+Ad[1,2]*(z2-z3-(dz2-dz3))+Ad[1,3]*(z2-z4-(dz2-dz4))+A0[1,1]*(z2-z0-dz2)
		e_3=Ad[2,0]*(z3-z1-(dz3-dz1))+Ad[2,1]*(z3-z2-(dz3-dz2))+Ad[2,3]*(z3-z4-(dz3-dz4))+A0[2,2]*(z3-z0-dz3)
		e_4=Ad[3,0]*(z4-z1-(dz4-dz1))+Ad[3,1]*(z4-z2-(dz4-dz2))+Ad[3,2]*(z4-z3-(dz4-dz3))+A0[3,3]*(z4-z0-dz4)

###############################Formation X-Y###################
		dx1=np.array([[4.0],[0.]])
		dx2=np.array([[-4.0],[0.]])
		dx3=np.array([[4.0],[0.]])
		dx4=np.array([[-4.0],[0.]])

		dy1=np.array([[-0.0],[0.]])
		dy2=np.array([[0.0],[0.]])
		dy3=np.array([[0.0],[0.]])
		dy4=np.array([[-0.0],[0.]])


		x0=np.array([[cnt_0.pos_uav.x],[cnt_0.vel_uav.x]])
		x1=np.array([[cnt_1.pos_uav.x],[cnt_1.vel_uav.x]])
		x2=np.array([[cnt_2.pos_uav.x],[cnt_2.vel_uav.x]])
		x3=np.array([[cnt_3.pos_uav.x],[cnt_3.vel_uav.x]])
		x4=np.array([[cnt_4.pos_uav.x],[cnt_4.vel_uav.x]])

		e_x1=Ad[0,1]*(x1-x2-(dx1-dx2))+Ad[0,2]*(x1-x3-(dx1-dx3))+Ad[0,3]*(x1-x4-(dx1-dx4))+A0[0,0]*(x1-x0-dx1)
		e_x2=Ad[1,0]*(x2-x1-(dx2-dx1))+Ad[1,2]*(x2-x3-(dx2-dx3))+Ad[1,3]*(x2-x4-(dx2-dx4))+A0[1,1]*(x2-x0-dx2)
		e_x3=Ad[2,0]*(x3-x1-(dx3-dx1))+Ad[2,1]*(x3-x2-(dx3-dx2))+Ad[2,3]*(x3-x4-(dx3-dx4))+A0[2,2]*(x3-x0-dx3)
		e_x4=Ad[3,0]*(x4-x1-(dx4-dx1))+Ad[3,1]*(x4-x2-(dx4-dx2))+Ad[3,2]*(x4-x3-(dx4-dx3))+A0[3,3]*(x4-x0-dx4)
		
		y0=np.array([[cnt_0.pos_uav.y],[cnt_0.vel_uav.y]])
		y1=np.array([[cnt_1.pos_uav.y],[cnt_1.vel_uav.y]])
		y2=np.array([[cnt_2.pos_uav.y],[cnt_2.vel_uav.y]])
		y3=np.array([[cnt_3.pos_uav.y],[cnt_3.vel_uav.y]])
		y4=np.array([[cnt_4.pos_uav.y],[cnt_4.vel_uav.y]])

		e_y1=Ad[0,1]*(y1-y2-(dy1-dy2))+Ad[0,2]*(y1-y3-(dy1-dy3))+Ad[0,3]*(y1-y4-(dy1-dy4))+A0[0,0]*(y1-y0-dy1)
		e_y2=Ad[1,0]*(y2-y1-(dy2-dy1))+Ad[1,2]*(y2-y3-(dy2-dy3))+Ad[1,3]*(y2-y4-(dy2-dy4))+A0[1,1]*(y2-y0-dy2)
		e_y3=Ad[2,0]*(y3-y1-(dy3-dy1))+Ad[2,1]*(y3-y2-(dy3-dy2))+Ad[2,3]*(y3-y4-(dy3-dy4))+A0[2,2]*(y3-y0-dy3)
		e_y4=Ad[3,0]*(y4-y1-(dy4-dy1))+Ad[3,1]*(y4-y2-(dy4-dy2))+Ad[3,2]*(y4-y3-(dy4-dy3))+A0[3,3]*(y4-y0-dy4)
		
		#pdb.set_trace()
		F=cnt_1.brf_z(e_1,u1,k_fz,lamz,Gammaz,ktez)
		Fx1, Fy1 = cnt_1.brf(e_x1,e_y1,ux1,uy1,k_f,lam,Gamma,kte)

		F2=cnt_2.brf_z(e_2,u2,k_fz,lamz,Gammaz,ktez)
		Fx2, Fy2 = cnt_2.brf(e_x2,e_y2,ux2,uy2,k_f,lam,Gamma,kte)

		F3=cnt_3.brf_z(e_3,u3,k_fz,lamz,Gammaz,ktez)
		Fx3, Fy3 = cnt_3.brf(e_x3,e_y3,ux3,uy3,k_f,lam,Gamma,kte)
		
		F4=cnt_4.brf_z(e_4,u4,k_fz,lamz,Gammaz,ktez)
		Fx4, Fy4 = cnt_4.brf(e_x4,e_y4,ux4,uy4,k_f,lam,Gamma,kte)

		u1=cop*np.dot(K_z,e_1)-F
		ux1=cop*np.dot(K,e_x1)-Fx1
		uy1=cop*np.dot(K,e_y1)-Fy1

		u2=cop*np.dot(K_z,e_2)-F2
		ux2=cop*np.dot(K,e_x2)-Fx2
		uy2=cop*np.dot(K,e_y2)-Fy2
		
		u3=cop*np.dot(K_z,e_3)-F3
		ux3=cop*np.dot(K,e_x3)-Fx3
		uy3=cop*np.dot(K,e_y3)-Fy3
		
		u4=cop*np.dot(K_z,e_4)-F4
		ux4=cop*np.dot(K,e_x4)-Fx4
		uy4=cop*np.dot(K,e_y4)-Fy4

		print(F,Fx1,Fy1)
		#pdb.set_trace()	
		cnt_1.updateSp(u1,ux1,uy1)
		cnt_2.updateSp(u2,ux2,uy2)
		cnt_3.updateSp(u3,ux3,uy3)
		cnt_4.updateSp(u4,ux4,uy4)

		at_pub0.publish(cnt_0.at)
	    	at_pub1.publish(cnt_1.at)
	 	at_pub2.publish(cnt_2.at)
		at_pub3.publish(cnt_3.at)
		at_pub4.publish(cnt_4.at)
	else:
		modes_0.setAutoLandMode()
		modes_1.setAutoLandMode()
		modes_2.setAutoLandMode()
		modes_3.setAutoLandMode()
		modes_4.setAutoLandMode()
	time+=dt
	print(time)		
	rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
