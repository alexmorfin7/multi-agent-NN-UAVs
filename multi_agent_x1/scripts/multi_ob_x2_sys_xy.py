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
from math import atan2, sqrt, pi, cos, sin
#import quaternion transformation
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import uavVModulexy2 as uavV
import uav1Modulexy2 as uav1
import uav2Modulexy2 as uav2

	
import pdb
N = 3
copz=2.5#.5
cop=14.7#.5
ref_z=3.0
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
Gamma=70
kte=0.003
#u1=np.zeros((1,1))
# Main function
def main():
    ###############intial values########
    Ad = np.array([[0.,1.],[1.,0.]])
    D = np.array([[1.,0.],[0.,1.]])
    A0 = np.array([[0.,0.],[0.,1.]])
    K_z = np.array([[-0.75,-0.35]])#-0.975,-0.293 
    Kx = np.array([[-0.015,-0.02]])#0.020,0.10
    Ky = np.array([[-0.019,-0.02]])
    x_init = np.array([[-3.,0.]])#np.array([[3.,0.,3.,0.]])
    y_init = np.array([[0.,3.]])#np.array([[0.,3.,3.,0.]])
    z_init = np.array([[4.,4.]])
    # initiate node
    rospy.init_node('multi_test_sys_xy_2', anonymous=True)
#Archivo donde indica el numero de prueba
    last=open('/home/alexmorfin/catkin_ws/src/multi_agent_x1/scripts/Pruebas_multi_sys_xy_2/last.txt','r')
    archivo=last.read()
    archivo=int(archivo)+1
    last=open('/home/alexmorfin/catkin_ws/src/multi_agent_x1/scripts/Pruebas_multi_sys_xy_2/last.txt','w')
    last.write(str(archivo))
    last.close()
    d_csv={}
    e={}
    #Crear Archivo de datos o de prueba
    for i in range(N):	
	d_csv['f_csv{}'.format(i)]=open('/home/alexmorfin/catkin_ws/src/multi_agent_x1/scripts/Pruebas_multi_sys_xy_2/prueba_%s_uav%s.txt'%(archivo,i),'a')
        d_csv['f_csv{}'.format(i)].write('time, uz, alt, vel_alt, uthe, roll, uphi, pitch, upsi, yaw, ux, pos_x, vel_x, uy, pos_y, vel_y, Fz, Fx, Fy'+'\n')
	d_csv['f_csv{}'.format(i)].close()
##UAV leader#########################

    cnt_0 = uavV.Controller(archivo)


############UAV 1####################################
    # flight mode object
    modes_1 = uav1.fcuModes()
    # controller object
    cnt_1 = uav1.Controller(archivo,x_init[0,0],y_init[0,0],z_init[0,0])
    rate = rospy.Rate(1/cnt_1.dt)
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
    cnt_2 = uav2.Controller(archivo,x_init[0,1],y_init[0,1],z_init[0,1])

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


    # Make sure the drone is armed
    while not (cnt_1.state.armed and cnt_2.state.armed):

	modes_1.setArm()
	modes_2.setArm()

	rate.sleep()



    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    k=0
    while k<10:

	at_pub1.publish(cnt_1.at)
	at_pub2.publish(cnt_2.at)

        rate.sleep()
        k = k + 1

    # activate OFFBOARD mode

    modes_1.setOffboardMode()
    modes_2.setOffboardMode()


    u1=0.0
    ux1=0.0
    uy1=0.0
    u2=0.0
    ux2=0.0
    uy2=0.0
    
    time=0.0
    dt=0.05
    off_x=0.0#1.5
    off_y=0.0#1.5
    r=15.0
    #F=0.0
    # ROS main loop
    while not rospy.is_shutdown():
	if (time<=5.0):
		#sp_pub0.publish(cnt_0.sp)
		sp_pub1.publish(cnt_1.sp)
		sp_pub2.publish(cnt_2.sp)
		
	elif (time>5.0 and time<=65.0):
		ref_x=(-r*cos((time-5.)*(3.1416*8/180))+r)+off_x#1.0#(-3*cos((time-5.)*(3.1416*8/180))+3.)+off_x
		ref_xp=(r*sin((time-5.)*(3.1416*8/180))+r)*dt
		ref_y=(-r*sin((time-5.)*(3.1416*8/180)))+off_y#2*sin((time-5.0)/6.5)#(-3*sin((time-5.)*(3.1416*8/180)))+off_y
		ref_yp=(-r*cos((time-5.)*(3.1416*8/180)))*dt#2*cos((time-5.0)/6.5)*(dt)
		ref_z=4.0#6-3*cos((time-5.0)/6.5)
		ref_zp=0.#3*sin((time-5.0)/6.5)*((dt))

		cnt_0.data(ref_x,ref_xp,ref_y,ref_yp,ref_z,ref_zp)

	#########Calculos#########################
	###############estados###########
		z0=np.array([[ref_z],[ref_zp]])
		z1=np.array([[cnt_1.pos_uav.z],[cnt_1.vel_uav.z]])
		z2=np.array([[cnt_2.pos_uav.z],[cnt_2.vel_uav.z]])

		e_1=Ad[0,1]*(z1-z2)+A0[0,0]*(z1-z0)
		e_2=Ad[1,0]*(z2-z1)+A0[1,1]*(z2-z0)


###############################Formation X-Y###################
		dx1=np.array([[0.0],[0.]])
		dx2=np.array([[0.0],[0.]])


		dy1=np.array([[0.0],[0.]])
		dy2=np.array([[0.0],[0.]])

############################Horizontal error##########################

		x0=np.array([[ref_x],[ref_xp]])
		x1=np.array([[cnt_1.pos_uav.x],[cnt_1.vel_uav.x]])
		x2=np.array([[cnt_2.pos_uav.x],[cnt_2.vel_uav.x]])

		e_x1=Ad[0,1]*(x1-x2)+A0[0,0]*(x1-x0)
		e_x2=Ad[1,0]*(x2-x1)+A0[1,1]*(x2-x0)

		y0=np.array([[ref_y],[ref_yp]])
		y1=np.array([[cnt_1.pos_uav.y],[cnt_1.vel_uav.y]])
		y2=np.array([[cnt_2.pos_uav.y],[cnt_2.vel_uav.y]])

		e_y1=Ad[0,1]*(y1-y2)+A0[0,0]*(y1-y0)
		e_y2=Ad[1,0]*(y2-y1)+A0[1,1]*(y2-y0)

		
		#pdb.set_trace()
		F=cnt_1.brf_z(e_1,u1,k_fz,lamz,Gammaz,ktez)
		Fx1, Fy1 = cnt_1.brf(e_x1,e_y1,ux1,uy1,k_f,lam,Gamma,kte)

		F2=cnt_2.brf_z(e_2,u2,k_fz,lamz,Gammaz,ktez)
		Fx2, Fy2 = cnt_2.brf(e_x2,e_y2,ux2,uy2,k_f,lam,Gamma,kte)


		u1=copz*np.dot(K_z,e_1)-F
		ux1=cop*np.dot(Kx,e_x1)-Fx1
		uy1=cop*np.dot(Ky,e_y1)-Fy1

		u2=copz*np.dot(K_z,e_2)-F2
		ux2=cop*np.dot(Kx,e_x2)-Fx2
		uy2=cop*np.dot(Ky,e_y2)-Fy2
		

		print(F,Fx1,Fy1)
		#pdb.set_trace()	
		cnt_1.updateSp(u1,ux1,uy1)
		cnt_2.updateSp(u2,ux2,uy2)

	    	at_pub1.publish(cnt_1.at)
	 	at_pub2.publish(cnt_2.at)

	else:

		modes_1.setAutoLandMode()
		modes_2.setAutoLandMode()

	time+=dt
	print(time)		
	rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
