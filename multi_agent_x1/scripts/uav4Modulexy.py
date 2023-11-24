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
import numpy as np
#import pdb
# Flight modes class
# Flight modes are activated using ROS services
#global archivo
class fcuModes:
    def __init__(self):
        pass

    def setTakeoff(self):
    	rospy.wait_for_service('uav4/mavros/cmd/takeoff')
    	try:
    		takeoffService = rospy.ServiceProxy('uav4/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
    		takeoffService(altitude = 3)
    	except rospy.ServiceException, e:
    		print "Service takeoff call failed: %s"%e

    def setArm(self):
        rospy.wait_for_service('uav4/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('uav4/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e

    def setDisarm(self):
        rospy.wait_for_service('uav4/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('uav4/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException, e:
            print "Service disarming call failed: %s"%e

    def setStabilizedMode(self):
        rospy.wait_for_service('uav4/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('uav4/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Stabilized Mode could not be set."%e

    def setOffboardMode(self):
        rospy.wait_for_service('uav4/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('uav4/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e

    def setAltitudeMode(self):
        rospy.wait_for_service('uav4/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('uav4/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Altitude Mode could not be set."%e

    def setPositionMode(self):
        rospy.wait_for_service('uav4/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('uav4/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Position Mode could not be set."%e

    def setAutoLandMode(self):
        rospy.wait_for_service('uav4/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('uav4/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException, e:
               print "service set_mode call failed: %s. Autoland Mode could not be set."%e

class Controller:
    # initialization method
    def __init__(self, archivo = 0,x_init=0.,y_init=0.,z_init=0.):
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
        self.sp.position.x = float(x_init)
        self.sp.position.y = float(y_init)
	self.sp.position.z = float(z_init) 
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
	self.nn=25
	self.nnxy=49

        #archivo
	self.archivo=archivo
	
	self.brfz= np.zeros((self.nn,1))
	self.brfzf= np.zeros((self.nn,1))
	self.sigmazf=np.zeros((2,1))
	self.deltazf=np.zeros((2,1))
	self.Pz=np.zeros((self.nn,self.nn))
	self.Qz=np.zeros((self.nn,1))
	self.wz=np.zeros((self.nn,1))
	self.brfx= np.zeros((self.nnxy,1))
	self.brfxf= np.zeros((self.nnxy,1))
	self.sigmaxf=np.zeros((2,1))
	self.deltaxf=np.zeros((2,1))
	self.Px=np.zeros((self.nnxy,self.nnxy))
	self.Qx=np.zeros((self.nnxy,1))
	self.wx=np.random.rand(self.nnxy,1)
	self.brfy= np.zeros((self.nnxy,1))
	self.brfyf= np.zeros((self.nnxy,1))
	self.sigmayf=np.zeros((2,1))
	self.deltayf=np.zeros((2,1))
	self.Py=np.zeros((self.nnxy,self.nnxy))
	self.Qy=np.zeros((self.nnxy,1))
	self.wy=np.random.rand(self.nnxy,1)
	
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
    def data(self,ux,uy):
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
 	Fz=np.dot(self.wz.transpose(),self.brfz)
	Fx=np.dot(self.wx.transpose(),self.brfx)
	Fy=np.dot(self.wy.transpose(),self.brfy)
	pos_x=self.pos_uav.x
	vel_x=self.vel_uav.x
	pos_y=self.pos_uav.y
	vel_y=self.vel_uav.y
	self.file_csv=open('/home/alexmorfin/catkin_ws/src/multi_agent_x1/scripts/Pruebas_multi_sys_xy/prueba_%s_uav4.txt'%(self.archivo),'a')
	self.datos=str([t, uz, alt, vel_alt, uthe, roll, uphi, pitch, upsi, yaw, float(ux), pos_x, vel_x, float(uy), pos_y, vel_y, float(Fz), float(Fx), float(Fy)])
	self.file_csv.write(self.datos+'\n')
	self.file_csv.close()
## Update setpoint message
    def updateSp(self,u,ux,uy):
       	#print(self.roll, self.pitch, self.yaw)
	alt=self.pos_uav.z
	vel_alt=self.vel_uav.z
	self.yaw_d = 0.0#numpy.deg2rad(180.0)
	self.pitch_d = atan((float(ux)*cos(self.yaw_d)+float(uy)*sin(self.yaw_d))/1)
	self.roll_d = atan(-(float(ux)*sin(self.yaw_d)+float(uy)*cos(self.yaw_d))*cos(self.pitch_d)/1)#0.0
	q = quaternion_from_euler(self.roll_d, self.pitch_d, self.yaw_d)
        self.at_des.pose.orientation = Quaternion(*q)
	#print(self.pitch_d,self.roll_d)
	self.at.orientation = self.at_des.pose.orientation
        self.at.thrust = float(u)/self.a3+0.55#+self.a2*alt+self.a1*vel_alt
	self.data(ux,uy)
	#print(self.at.thrust,float(ux),float(uy))
	#pdb.set_trace()
	#print(self.time,float(u1))
	self.time+=self.dt
##Red Neuronal
    def brf_z(self,ez,uz,k_fz,lamz,Gammaz,ktez):
	nn=25	
	T=self.dt
	AS = np.array([[0.0,1.0],[-0.0795,-0.251]])
	BS = np.array([[0.0],[9.268]])
	P_s = np.array([[0.1305,0.0351],[0.0351,0.0475]])
	#c=np.array([[-1.5,-1.5,-1.5,0.0,0.0,0.0,1.5,1.5,1.5],[-1.5,0.0,1.5,-1.5,0.0,1.5,-1.5,0.0,1.5]])
	c=np.array([[-3.0,-3.0,-3.0,-3.0,-3.0,-1.5,-1.5,-1.5,-1.5,-1.5,0.0,0.0,0.0,0.0,0.0,1.5,1.5,1.5,1.5,1.5,3.0,3.0,3.0,3.0,3.0],[-3.0,-1.5,0.0,1.5,3.0,-3.0,-1.5,0.0,1.5,3.0,-3.0,-1.5,0.0,1.5,3.0,-3.0,-1.5,0.0,1.5,3.0,-3.0,-1.5,0.0,1.5,3.0]])	
	#c=np.array([[-4.5 ,-4.5 ,-4.5 ,-4.5 ,-4.5 ,-4.5 ,-4.5, -3. ,-3. ,-3. ,-3. ,-3. ,-3. ,-3. ,-1.5 ,-1.5 ,-1.5 ,-1.5 ,-1.5 ,-1.5 ,-1.5 ,0. ,0. ,0. ,0. ,0. ,0. ,0. ,1.5 ,1.5 ,1.5 ,1.5 ,1.5 ,1.5 ,1.5 ,3. ,3. ,3. ,3. ,3. ,3. ,3. ,4.5 ,4.5 ,4.5 ,4.5 ,4.5 ,4.5 ,4.5], [-4.5 ,-3. ,-1.5 ,0. ,1.5 ,3. ,4.5 ,-4.5 ,-3.0 ,-1.5 ,0. ,1.5 ,3. ,4.5 ,-4.5 ,-3. ,-1.5 ,0. ,1.5 ,3. ,4.5,-4.5 ,-3. ,-1.5 ,0. ,1.5 ,3. ,4.5 ,-4.5 ,-3. ,-1.5 ,0. ,1.5 ,3. ,4.5,-4.5 ,-3. ,-1.5 ,0. ,1.5 ,3. ,4.5,-4.5 ,-3. ,-1.5 ,0. ,1.5 ,3. ,4.5]])	
	b=0.7
	inp_b=np.array([[self.pos_uav.z],[(self.vel_uav.z)*0.1]])
	#print(inp_b)
    	#pdb.set_trace()
    	for j in range(nn):
        	self.brfz[j]=(np.exp(-np.linalg.norm(inp_b-c[:,j])**2)/(2*b*b))
    	for n in range(nn):
		self.brfzf[n]+=(T/k_fz)*((self.brfz[n]) - self.brfzf[n] )	
	
	#print(self.brf1)
    	#pdb.set_trace()
	#print(self.brf1f)
    	#pdb.set_trace()
	sigmaz=np.dot(AS,ez)+np.dot(BS,uz)
	self.sigmazf += (T/k_fz)*( sigmaz - self.sigmazf)
	self.deltazf+=(T/k_fz)*(ez-self.deltazf)
	#print(sigma1)
	#print(self.sigma1f)
	#print(self.delta1f)
    	#pdb.set_trace()
####### Matrix P ##############		
	P_brfzf=np.dot(self.brfzf,self.brfzf.transpose())

	for p in range(nn):
        	for m in range(nn):
            		self.Pz[p,m]+= T*(-lamz*self.Pz[p,m] + P_brfzf[p,m])
	#print(P_brf1f)
    	#pdb.set_trace()
	#print(self.P1)
    	#pdb.set_trace()
####### Matrix Q #############
	psi=(ez - self.deltazf)/k_fz - self.sigmazf
	#print(psi)
    	#pdb.set_trace()
	for r in range(nn):
                self.Qz[r]+= T*(-lamz*self.Qz[r]+ self.brfzf[r]*(psi[1]))
	#print(self.Q1)
    	#pdb.set_trace()
####### Vector H_brf #########
	H_brfz=np.dot(self.Pz,self.wz)-self.Qz
	#print(H_brf1)
    	#pdb.set_trace()
############ uPDATE LAW ####
	self.wz+=T*(Gammaz*(np.dot(np.dot(self.brfz,ez.transpose()),np.dot(P_s,BS))-ktez*H_brfz))
        #print(self.w1)
    	#pdb.set_trace()         
        Fz = np.dot(self.wz.transpose(),self.brfz)
	#print(F1)
    	#pdb.set_trace()
	#print(F1)
	return Fz
############################### Neural Network ##################################
    def brf(self,e_x,e_y,ux,uy,k_f,lam,Gamma,kte):
	nn=25	
	T=self.dt
	AS = np.array([[0.0,1.0],[-0.020,-0.015]])
	BS = np.array([[0.0],[1]])
	P_s = np.array([[0.2267,0.1742],[0.1742,0.5362]])
	#c=np.array([[-0.5,-0.5,-0.5,0.0,0.0,0.0,0.5,0.5,0.5],[-0.5,0.0,0.5,-0.5,0.0,0.5,-0.5,0.0,0.5]])
	#c=np.array([[-3.0,-3.0,-3.0,-3.0,-3.0,-1.5,-1.5,-1.5,-1.5,-1.5,0.0,0.0,0.0,0.0,0.0,1.5,1.5,1.5,1.5,1.5,3.0,3.0,3.0,3.0,3.0],[-3.0,-1.5,0.0,1.5,3.0,-3.0,-1.5,0.0,1.5,3.0,-3.0,-1.5,0.0,1.5,3.0,-3.0,-1.5,0.0,1.5,3.0,-3.0,-1.5,0.0,1.5,3.0]])	
	c=np.array([[-30.,-30.,-30.,-30.,-30.,-15.,-15.,-15.,-15.,-15.,0.0,0.0,0.0,0.0,0.0,15.,15.,15.,15.,15.,30.,30.,30.,30.,30.],[-30.,-15.,0.0,15.,30.,-30,-15.,0.0,15.,30.,-30.,-15.,0.0,15.,30.,-30.,-15.,0.0,15.,30.,-30.,-15.,0.0,15.,30.]])	
	#c=np.array([[-4.5 ,-4.5 ,-4.5 ,-4.5 ,-4.5 ,-4.5 ,-4.5, -3. ,-3. ,-3. ,-3. ,-3. ,-3. ,-3. ,-1.5 ,-1.5 ,-1.5 ,-1.5 ,-1.5 ,-1.5 ,-1.5 ,0. ,0. ,0. ,0. ,0. ,0. ,0. ,1.5 ,1.5 ,1.5 ,1.5 ,1.5 ,1.5 ,1.5 ,3. ,3. ,3. ,3. ,3. ,3. ,3. ,4.5 ,4.5 ,4.5 ,4.5 ,4.5 ,4.5 ,4.5], [-4.5 ,-3. ,-1.5 ,0. ,1.5 ,3. ,4.5 ,-4.5 ,-3.0 ,-1.5 ,0. ,1.5 ,3. ,4.5 ,-4.5 ,-3. ,-1.5 ,0. ,1.5 ,3. ,4.5,-4.5 ,-3. ,-1.5 ,0. ,1.5 ,3. ,4.5 ,-4.5 ,-3. ,-1.5 ,0. ,1.5 ,3. ,4.5,-4.5 ,-3. ,-1.5 ,0. ,1.5 ,3. ,4.5,-4.5 ,-3. ,-1.5 ,0. ,1.5 ,3. ,4.5]])
	b=9.
	inp_x=np.array([[self.pos_uav.x*.1],[(self.vel_uav.x)*.1]])
	inp_y=np.array([[self.pos_uav.y*.1],[(self.vel_uav.y)*.1]])
	#print(inp_b)
    	#pdb.set_trace()
    	for j in range(nn):
        	self.brfx[j]=(np.exp(-np.linalg.norm(inp_x-c[:,j])**2)/(2*b*b))
		self.brfy[j]=(np.exp(-np.linalg.norm(inp_y-c[:,j])**2)/(2*b*b))
    	for n in range(nn):
		self.brfxf[n]+=(T/k_f)*((self.brfx[n]) - self.brfxf[n] )	
		self.brfyf[n]+=(T/k_f)*((self.brfy[n]) - self.brfyf[n] )	
	
	#print(self.brf1)
    	#pdb.set_trace()
	#print(self.brf1f)
    	#pdb.set_trace()
	sigmax=np.dot(AS,e_x)+np.dot(BS,ux)
	sigmay=np.dot(AS,e_y)+np.dot(BS,uy)
	
	self.sigmaxf += (T/k_f)*( sigmax - self.sigmaxf)
	self.sigmayf += (T/k_f)*( sigmay - self.sigmayf)	
	self.deltaxf+=(T/k_f)*(e_x-self.deltaxf)
	self.deltayf+=(T/k_f)*(e_y-self.deltayf)
	#print(sigma1)
	#print(self.sigma1f)
	#print(self.delta1f)
    	#pdb.set_trace()
####### Matrix P ##############		
	P_brfxf=np.dot(self.brfxf,self.brfxf.transpose())
	P_brfyf=np.dot(self.brfyf,self.brfyf.transpose())

	for p in range(nn):
        	for m in range(nn):
            		self.Px[p,m]+= T*(-lam*self.Px[p,m] + P_brfxf[p,m])
			self.Py[p,m]+= T*(-lam*self.Py[p,m] + P_brfyf[p,m])
	#print(P_brf1f)
    	#pdb.set_trace()
	#print(self.P1)
    	#pdb.set_trace()
####### Matrix Q #############
	psix=(e_x - self.deltaxf)/k_f - self.sigmaxf
	psiy=(e_y - self.deltayf)/k_f - self.sigmayf
	#print(psix)
    	#pdb.set_trace()
	for r in range(nn):
                self.Qx[r]+= T*(-lam*self.Qx[r]+ self.brfxf[r]*(psix[1]))
		self.Qy[r]+= T*(-lam*self.Qy[r]+ self.brfyf[r]*(psiy[1]))
	#print(self.Q1)
    	#pdb.set_trace()
####### Vector H_brf #########
	H_brfx=np.dot(self.Px,self.wx)-self.Qx
	H_brfy=np.dot(self.Py,self.wy)-self.Qy
	#print(H_brf1)
    	#pdb.set_trace()
############ uPDATE LAW ####
	self.wx+=T*(Gamma*(np.dot(np.dot(self.brfx,e_x.transpose()),np.dot(P_s,BS))-kte*H_brfx))
	self.wy+=T*(Gamma*(np.dot(np.dot(self.brfy,e_y.transpose()),np.dot(P_s,BS))-kte*H_brfy))        
	#print(self.w1)
    	#pdb.set_trace()         
        Fx = np.dot(self.wx.transpose(),self.brfx)*1.
	Fy = np.dot(self.wy.transpose(),self.brfy)*1.
	#print(F1)
    	#pdb.set_trace()
	#print(F1)
	return Fx, Fy

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
