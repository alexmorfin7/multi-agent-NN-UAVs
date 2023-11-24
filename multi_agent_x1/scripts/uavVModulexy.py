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
from math import atan, atan2, sqrt, pi, cos, sin
#import quaternion transformation
from tf.transformations import euler_from_quaternion, quaternion_from_euler
#from numpy import *
# Flight modes class
# Flight modes are activated using ROS services
#global archivo

class Controller:
    # initialization method
    def __init__(self, archivo = 0):
        # Drone state
        self.state = State()
        # Instantiate a attitude_des message
        self.at = AttitudeTarget()
	# Instantiate a setpoints message
        self.sp = PositionTarget()
	self.sp.type_mask = int('010111111000', 2)
        # LOCAL_NED
        self.sp.coordinate_frame = 1
	# initial values for setpoints
        self.sp.position.x = 1.5
        self.sp.position.y = 1.5
	self.sp.position.z = 3.0
        # set the flag to use position setpoints and yaw angle
        self.at.type_mask = int('00111', 2)#7
	###########
        self.at_des = PoseStamped()
	self.at_des.header.stamp = rospy.Time.now() 
	self.at_des.pose.position.x = 0.0
        self.at_des.pose.position.y = 0.0
        self.at_des.pose.position.z = 0.0
	self.at_des.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        
        # Altitude and yaw reference, [meters] [rad]
        self.alt_des = 1.0
	self.yaw_des = 0.0
        self.a1=0.251
	self.a2=0.0795
	self.a3=9.268
	
        # A Message for the current local position of the drone
        self.pos_uav = Point(0.0, 0.0, 0.0)
	self.att_uav = Quaternion(0.0, 0.0, 0.0, 1.0)
	self.vel_uav = Vector3(0.0, 0.0, 0.0)
	self.ang_vel_uav = Vector3(0.0, 0.0, 0.0)

        # initial values for attitude references
        self.at.orientation.x = 0.0
        self.at.orientation.y = 0.0
	self.at.orientation.z = 0.0
        self.at.orientation.w = 1.0
	self.at.thrust = 0.0
	#euler angle
	self.roll = 0.0
	self.pitch = 0.0
	self.yaw = 0.0
	#euler angle reference
	self.roll_d = 0.0
	self.pitch_d = 0.0
	self.yaw_d = 0.0
	self.time=0.0
	self.dt=0.05
        #archivo
	self.archivo=archivo

## Data ##
    def data(self,ref_x,ref_xp,ref_y,ref_yp,ref_z,ref_zp):
	t=self.time
	alt=ref_z
	vel_alt=ref_zp
	pos_x=ref_x
	pos_y=ref_y
	uz=0.0
	uthe=0.0
	roll=0.0
	pitch=0.0
	upsi=0.0
	yaw=0.0
	ux=0.0
	vel_x=ref_xp
	uy=0.0
	vel_y=ref_yp
	uphi=0.0
	self.file_csv=open('/home/alexmorfin/catkin_ws/src/multi_agent_x1/scripts/Pruebas_multi_sys_xy/prueba_%s_uav0.txt'%(self.archivo),'a')
	self.datos=str([t, uz, alt, vel_alt, uthe, roll, uphi, pitch, upsi, yaw, ux, pos_x, vel_x, uy, pos_y, vel_y])
	self.file_csv.write(self.datos+'\n')
	self.file_csv.close()
	self.time+=self.dt
## Update setpoint message

# Main function
def main():
    global archivo
    # initiate node
    rospy.init_node('test_multi_sys_z', anonymous=True)

    # flight mode object
    modes = fcuModes()

    # controller object
    cnt = Controller()

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
    #Archivo donde indica el numero de prueba
    last=open('/home/alexmorfin/catkin_ws/src/multi_agent_x1/scripts/Pruebas_multi_sys/last.txt','r')
    archivo=last.read()
    archivo=int(archivo)+1
    last=open('/home/alexmorfin/catkin_ws/src/multi_agent_x1/scripts/Pruebas_multi_sys/last.txt','w')
    last.write(str(archivo))
    last.close()
	
    #Crear Archivo de datos o de prueba
    f_csv=open('/home/alexmorfin/catkin_ws/src/multi_agent_x1/scripts/Pruebas_multi_sys/prueba_%s.txt'%(archivo),'a')
    f_csv.write('time, uz, alt, vel_alt, uthe, roll, uphi, pitch, upsi, yaw'+'\n')

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
