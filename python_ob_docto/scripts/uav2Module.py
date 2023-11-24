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
import numpy as np
#import pdb
# Flight modes class
# Flight modes are activated using ROS services
#global archivo
class fcuModes:
    def __init__(self):
        pass

    def setTakeoff(self):
    	rospy.wait_for_service('uav2/mavros/cmd/takeoff')
    	try:
    		takeoffService = rospy.ServiceProxy('uav2/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
    		takeoffService(altitude = 3)
    	except rospy.ServiceException, e:
    		print "Service takeoff call failed: %s"%e

    def setArm(self):
        rospy.wait_for_service('uav2/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('uav2/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e

    def setDisarm(self):
        rospy.wait_for_service('uav2/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('uav2/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException, e:
            print "Service disarming call failed: %s"%e

    def setStabilizedMode(self):
        rospy.wait_for_service('uav2/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('uav2/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Stabilized Mode could not be set."%e

    def setOffboardMode(self):
        rospy.wait_for_service('uav2/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('uav2/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e

    def setAltitudeMode(self):
        rospy.wait_for_service('uav2/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('uav2/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Altitude Mode could not be set."%e

    def setPositionMode(self):
        rospy.wait_for_service('uav2/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('uav2/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Position Mode could not be set."%e

    def setAutoLandMode(self):
        rospy.wait_for_service('uav2/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('uav2/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException, e:
               print "service set_mode call failed: %s. Autoland Mode could not be set."%e

class Controller:
    # initialization method
    def __init__(self, archivo = 0):
        # Drone state
        self.state = State()
        # Instantiate a attitude_des message
        self.at = AttitudeTarget()
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
	
	self.brf1= np.zeros((9,1))
	self.brf1f= np.zeros((9,1))
	self.sigma1f=np.zeros((2,1))
	self.delta1f=np.zeros((2,1))
	self.P1=np.zeros((9,9))
	self.Q1=np.zeros((9,1))
	self.w1=np.zeros((9,1))
	#self.H_brf1=np.zeros((9,1))
# Callbacks

## POSE UAV callback
    def poseCb(self, msg):
        self.pos_uav.x = msg.pose.position.x
        self.pos_uav.y = msg.pose.position.y
        self.pos_uav.z = msg.pose.position.z

	self.att_uav.x = msg.pose.orientation.x
	self.att_uav.y = msg.pose.orientation.y
	self.att_uav.z = msg.pose.orientation.z
	self.att_uav.w = msg.pose.orientation.w
	
	self.roll, self.pitch, self.yaw = euler_from_quaternion([self.att_uav.x,self.att_uav.y,self.att_uav.z,self.att_uav.w])
	
## Vel UAV callback
    def velCb(self, msg):
        self.vel_uav.x = msg.twist.linear.x
        self.vel_uav.y = msg.twist.linear.y
        self.vel_uav.z = msg.twist.linear.z

	self.ang_vel_uav.x = msg.twist.angular.x
	self.ang_vel_uav.y = msg.twist.angular.y
	self.ang_vel_uav.z = msg.twist.angular.z
## keyboard callback
    def keyCb(self, msg):
        self.vel_uav.x = msg.twist.linear.x
        self.vel_uav.y = msg.twist.linear.y
        self.vel_uav.z = msg.twist.linear.z

	self.ang_vel_uav.x = msg.twist.angular.x
	self.ang_vel_uav.y = msg.twist.angular.y
	self.ang_vel_uav.z = msg.twist.angular.z
## Drone State callback
    def stateCb(self, msg):
        self.state = msg
## Data ##
    def data(self):
	t=self.time
	roll=self.roll
	pitch=self.pitch
	yaw=self.yaw
	alt=self.pos_uav.z
	vel_alt=self.vel_uav.z
	uz= self.at.thrust
	uthe=self.roll_d
	uphi=self.pitch_d
	upsi=self.yaw_d
 	F=np.dot(self.w1.transpose(),self.brf1)
	self.file_csv=open('/home/alexmorfin/catkin_ws/src/python_ob_docto/scripts/Pruebas_multi_sys/prueba_%s_uav2.txt'%(self.archivo),'a')
	self.datos=str([t, uz, alt, vel_alt, uthe, roll, uphi, pitch, upsi, yaw, F])
	self.file_csv.write(self.datos+'\n')
	self.file_csv.close()
## Update setpoint message
    def updateSp(self,u):
       	#print(self.roll, self.pitch, self.yaw)
	self.data()
	self.roll_d = 0.0
	self.pitch_d = 0.0
	self.yaw_d = 0.0#numpy.deg2rad(180.0)
	q = quaternion_from_euler(self.roll_d, self.pitch_d, self.yaw_d)
        self.at_des.pose.orientation = Quaternion(*q)
	#print(self.at_des.pose.orientation)
	self.at.orientation = self.at_des.pose.orientation
        self.at.thrust = float(u)
	#print(u1)
	#pdb.set_trace()
	#print(self.time,float(u1))
	self.time+=self.dt
##Red Neuronal
    def brf_z(self,e,u,k_f,lam,Gamma,kte):
	nn=9	
	T=self.dt
	AS = np.array([[0.0,1.0],[-0.0795,-0.251]])
	BS = np.array([[0.0],[9.268]])
	P_s = np.array([[0.1305,0.0351],[0.0351,0.0475]])
	c=np.array([[-0.5,-0.5,-0.5,0.0,0.0,0.0,0.5,0.5,0.5],[-0.5,0.0,0.5,-0.5,0.0,0.5,-0.5,0.0,0.5]])
	b=0.3
	inp_b=np.array([[self.pos_uav.z],[(self.vel_uav.z)*2.5]])
	#print(inp_b)
    	#pdb.set_trace()
    	for j in range(nn):
        	self.brf1[j]=(np.exp(-np.linalg.norm(inp_b-c[:,j])**2)/(2*b*b))
    	for n in range(nn):
		self.brf1f[n]+=(T/k_f)*((self.brf1[n]) - self.brf1f[n] )	
	
	#print(self.brf1)
    	#pdb.set_trace()
	#print(self.brf1f)
    	#pdb.set_trace()
	sigma1=np.dot(AS,e)+np.dot(BS,u)
	self.sigma1f += (T/k_f)*( sigma1 - self.sigma1f)
	self.delta1f+=(T/k_f)*(e-self.delta1f)
	#print(sigma1)
	#print(self.sigma1f)
	#print(self.delta1f)
    	#pdb.set_trace()
####### Matrix P ##############		
	P_brf1f=np.dot(self.brf1f,self.brf1f.transpose())

	for p in range(nn):
        	for m in range(nn):
            		self.P1[p,m]+= T*(-lam*self.P1[p,m] + P_brf1f[p,m])
	#print(P_brf1f)
    	#pdb.set_trace()
	#print(self.P1)
    	#pdb.set_trace()
####### Matrix Q #############
	psi=(e - self.delta1f)/k_f - self.sigma1f
	#print(psi)
    	#pdb.set_trace()
	for r in range(nn):
                self.Q1[r]+= T*(-lam*self.Q1[r]+ self.brf1f[r]*(psi[1]))
	#print(self.Q1)
    	#pdb.set_trace()
####### Vector H_brf #########
	H_brf1=np.dot(self.P1,self.w1)-self.Q1
	#print(H_brf1)
    	#pdb.set_trace()
############ uPDATE LAW ####
	self.w1+=T*(Gamma*(np.dot(np.dot(self.brf1,e.transpose()),np.dot(P_s,BS))-kte*H_brf1))
        #print(self.w1)
    	#pdb.set_trace()         
        F = np.dot(self.w1.transpose(),self.brf1)
	#print(F1)
    	#pdb.set_trace()
	#print(F1)
	return F

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
    last=open('/home/alexmorfin/catkin_ws/src/python_ob_docto/scripts/Pruebas_multi_sys/last.txt','r')
    archivo=last.read()
    archivo=int(archivo)+1
    last=open('/home/alexmorfin/catkin_ws/src/python_ob_docto/scripts/Pruebas_multi_sys/last.txt','w')
    last.write(str(archivo))
    last.close()
	
    #Crear Archivo de datos o de prueba
    f_csv=open('/home/alexmorfin/catkin_ws/src/python_ob_docto/scripts/Pruebas_multi_sys/prueba_%s.txt'%(archivo),'a')
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
