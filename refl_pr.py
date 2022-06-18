#!/usr/bin/env python3
import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from math import pow, atan2, sqrt, sin, cos, acos, asin, atan
from std_msgs.msg import String
from statistics import mean
from charge_init import Bus
class reflector_detect:	

    def __init__(self):
    	rospy.init_node('listener', anonymous = True)
    	rospy.Subscriber('/os_cloud_node/points', PointCloud2, self.reflector)
    	self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    	self.lift_publisher = rospy.Publisher('/cmd_lift', String, queue_size=10)
    	self.error_publisher = rospy.Publisher('/error_message', String, queue_size = 10)
    	self.error_resolve = rospy.Publisher('/error_resolve', String, queue_size = 10)
    	self.rate = rospy.Rate(10)
    	self.x_rfl0 = 100
    	self.y_rfl0 = 100
    	self.x_rfl1 = 170
    	self.y_rfl1 = 100
    	self.z_rfl0 = 0
    	self.z_rfl1 = 0
    	self.x_init
    	self.y_init 
    	self.x_init1 
    	self.y_init1 
    	self.tol = 0.5
    	self.stop_v = True
    	self.ems = False
    	#self.d = 20.6
    	
    def reflector(self, data):
    	pc = pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z","intensity"))
    	x_pts = []
    	y_pts = []
    	z_pts = []
    	x_pts1 = []
    	y_pts1 = []
    	z_pts1 = []
    	#print(self.x_ulim, self.x_dlim, self.y_ulim, self.y_dlim)
    	self.x_dlim = self.x_init - self.tol
    	self.x_ulim = self.x_init + self.tol
    	self.x_dlim1 = self.x_init1 - self.tol
    	self.x_ulim1 = self.x_init1 + self.tol
    	self.y_dlim = self.y_init - self.tol
    	self.y_ulim = self.y_init + self.tol
    	self.y_dlim1 = self.y_init1 - self.tol
    	self.y_ulim1 = self.y_init1 + self.tol
    	for p in pc:
    		if p[3] > 600 and (p[0] > self.x_dlim and p[0] < self.x_ulim) and (p[1] < self.y_ulim and p[1] > self.y_dlim):
    			x_pts.append(p[0])
    			y_pts.append(p[1])
    			z_pts.append(p[2])
    		if p[3] > 600 and (p[0] > self.x_dlim1 and p[0] < self.x_ulim1) and (p[1] < self.y_ulim1 and p[1] > self.y_dlim1):
    			x_pts1.append(p[0])
    			y_pts1.append(p[1])
    			z_pts1.append(p[2])
    	if x_pts and y_pts:
    		z_ds = mean(z_pts)
    		x_ds = mean(x_pts)
    		y_ds = mean(y_pts)
    		self.x_rfl0 = x_ds#*cos(atan(z_ds/x_ds))
    		self.y_rfl0 = y_ds#*cos(atan(z_ds/y_ds))
    		self.z_rfl0 = z_ds
    		print(self.x_rfl0, self.y_rfl0)
    		self.stop_v = False
    		self.error_resolve.publish("011")
    		self.ems = False
    	else:
    		print("no data")
    		self.stop_v = True
    		if self.ems == False:
    		    self.error_publisher.publish("011_Reflector1 Not Detected")
    		    self.ems = True
    		
    	if x_pts1 and y_pts1:
    		z_ds1 = mean(z_pts1)
    		x_ds1 = mean(x_pts1)
    		y_ds1 = mean(y_pts1)
    		self.x_rfl1 = x_ds1#*cos(atan(z_ds1/x_ds1))
    		self.y_rfl1 = y_ds1#*cos(atan(z_ds1/y_ds1))
    		self.z_rfl1 = z_ds1
    		print(self.x_rfl1, self.y_rfl1)
    		self.stop_v = False
    		self.ems = False
    		self.error_resolve.publish("012")
    	else:
    		print("no data")
    		self.stop_v = True
    		#self.error_publisher.publish("012_Reflector2 Not Detected")
    		if self.ems == False:
    		    self.error_publisher.publish("012_Reflector2 Not Detected")
    		    self.ems = True
    		
    	if self.x_rfl0 and self.y_rfl0:
    		if (self.x_rfl0 < self.x_ulim and self.x_rfl0 > self.x_dlim) and (self.y_rfl0 < self.y_ulim and self.y_rfl0 > self.y_dlim):
    			mv_erx = self.x_init - self.x_rfl0
    			self.x_dlim = self.x_dlim - mv_erx
    			self.x_ulim = self.x_ulim - mv_erx
    			self.x_init = self.x_init - mv_erx
    			mv_ery = self.y_init - self.y_rfl0
    			self.y_dlim = self.y_dlim - mv_ery
    			self.y_ulim = self.y_ulim - mv_ery
    			self.y_init = self.y_init - mv_ery
    			
    	if self.x_rfl1 and self.y_rfl1:
    		if (self.x_rfl1 < self.x_ulim1 and self.x_rfl1 > self.x_dlim1) and (self.y_rfl1 < self.y_ulim1 and self.y_rfl1 > self.y_dlim1):
    			mv_erx1 = self.x_init1 - self.x_rfl1
    			self.x_dlim1 = self.x_dlim1 - mv_erx1
    			self.x_ulim1 = self.x_ulim1 - mv_erx1
    			self.x_init1 = self.x_init1 - mv_erx1
    			mv_ery1 = self.y_init1 - self.y_rfl1
    			self.y_dlim1 = self.y_dlim1 - mv_ery1
    			self.y_ulim1 = self.y_ulim1 - mv_ery1
    			self.y_init1 = self.y_init1 - mv_ery1
    def euclidean_distance0(self,x_t,y_t):
    	d = sqrt(pow(self.y_rfl1 - self.y_rfl0,2)+pow(self.x_rfl1 - self.x_rfl0,2)+pow(self.z_rfl1-self.z_rfl0,2))
    	D = sqrt(pow(self.x_rfl0,2)+pow(self.y_rfl0,2)+pow(self.z_rfl0,2))
    	delta = sqrt(pow(self.x_rfl1,2)+pow(self.y_rfl1,2)+pow(self.z_rfl1,2))
    	alpha = acos(((d**2)+(D**2)-(delta**2))/(2*d*D))
    	x_s = D*cos(alpha)
    	y_s = D*sin(alpha)
    	dx = sqrt(pow(x_t - x_s,2)+pow(y_t - y_s,2))
    	print(x_s, y_s, d, dx)
    	if dx < 3:
    		self.tol = 0.3
    	return dx
    	
    def linear_vel0(self):
    	dx = self.euclidean_distance0()
    	if dx > 1:
    		lin_vel = 0.4
    	else:
    		lin_vel = 0.4*dx
    	return lin_vel
        		     	
    def angular_vel0(self, x_t, y_t, constant=1):
    	d = sqrt(pow(self.y_rfl1 - self.y_rfl0,2)+pow(self.x_rfl1 - self.x_rfl0,2)+pow(self.z_rfl1-self.z_rfl0,2))
    	D = sqrt(pow(self.x_rfl0,2)+pow(self.y_rfl0,2)+pow(self.z_rfl0,2))
    	delta = sqrt(pow(self.x_rfl1,2)+pow(self.y_rfl1,2)+pow(self.z_rfl1,2))
    	alpha = acos(((d**2)+(D**2)-(delta**2))/(2*d*D))
    	x_s = D*cos(alpha)
    	y_s = D*sin(alpha)
    	st = atan2(y_t - y_s, x_t - x_s)
    	v_ang = atan2(self.y_rfl1-self.y_rfl0,self.x_rfl1-self.x_rfl0)
    	ang_vel = v_ang-st 
    	if ang_vel>1.5:
    		ang_vel = 1.5
    	elif ang_vel <-1.5:
    		ang_vel = -1.5
    	else:
    		ang_vel = ang_vel
    	return ang_vel
    		
    def euclidean_distance1(self, x_t, y_t):
    	d = sqrt(pow(self.y_rfl1 - self.y_rfl0,2)+pow(self.x_rfl1 - self.x_rfl0,2)+pow(self.z_rfl1-self.z_rfl0,2))
    	D = sqrt(pow(self.x_rfl0,2)+pow(self.y_rfl0,2)+pow(self.z_rfl0,2))
    	delta = sqrt(pow(self.x_rfl1,2)+pow(self.y_rfl1,2)+pow(self.z_rfl1,2))
    	alpha = acos(((d**2)+(D**2)-(delta**2))/(2*d*D))
    	x_s = D*cos(alpha)
    	y_s = D*sin(alpha)
    	dx = sqrt(pow(x_t - x_s,2)+pow(y_t - y_s,2))
    	print(x_s, y_s, d, dx)
    	if dx < 3:
    		self.tol = 0.3
    	return dx
    	
    def linear_vel1(self):
    	dx = self.euclidean_distance1()
    	if dx > 1:
    		lin_vel = 0.4
    	else:
    		lin_vel = 0.4*dx
    	return lin_vel
        		     	
    def angular_vel1(self,x_t, y_t, constant=1):
    	d = sqrt(pow(self.y_rfl1 - self.y_rfl0,2)+pow(self.x_rfl1 - self.x_rfl0,2)+pow(self.z_rfl1-self.z_rfl0,2))
    	D = sqrt(pow(self.x_rfl0,2)+pow(self.y_rfl0,2)+pow(self.z_rfl0,2))
    	delta = sqrt(pow(self.x_rfl1,2)+pow(self.y_rfl1,2)+pow(self.z_rfl1,2))
    	alpha = acos(((d**2)+(D**2)-(delta**2))/(2*d*D))
    	x_s = D*cos(alpha)
    	y_s = D*sin(alpha)
    	st = atan2(y_t - y_s, x_t - x_s)
    	v_ang = atan2(self.y_rfl1-self.y_rfl0,self.x_rfl1-self.x_rfl0)
    	ang_vel = v_ang-st 
    	if ang_vel>1.5:
    		ang_vel = 1.5
    	elif ang_vel <-1.5:
    		ang_vel = -1.5
    	else:
    		ang_vel = ang_vel
    	return ang_vel
    
    def euclidean_distance2(self, Eud2):
    	d1 = sqrt(pow(self.x_rfl0,2) + pow(self.y_rfl0,2))
    	print('R3 distance = ', d1)
    	return abs(d1-Eud2)
    	
    def linear_vel2(self):
    	dx = self.euclidean_distance2()
    	if dx > 1:
    		lin_vel = 0.4
    	else:
    		lin_vel = 0.4*dx
    	return lin_vel
    	
    def angular_vel2(self):
    	st = atan2(self.x_rfl1 - self.x_rfl0, self.y_rfl1 - self.y_rfl0)
    	self.tol = 0.5
    	print(st)
    	ang_vel = -1.5*st #+ 1*ad
    	if ang_vel>1.5:
    		ang_vel = 1.5
    	elif ang_vel <-1.5:
    		ang_vel = -1.5
    	else:
    		ang_vel = ang_vel
    	return ang_vel
    	
    def euclidean_distance3(self, Eud3):
    	d2 = self.x_rfl1#sqrt(pow(self.y_rfl1,2) + pow(self.y_rfl1,2))
    	print('R2 distance = ', d2)
    	return abs(d2+Eud3)
    	
    	
    def linear_vel3(self):
    	dx = self.euclidean_distance3()
    	if dx > 1:
    		lin_vel = -0.5
    	else:
    		lin_vel = -0.5*dx
    	return lin_vel
    	
    def angular_vel3(self):
    	st = atan2(self.x_rfl1 - self.x_rfl0, self.y_rfl1 - self.y_rfl0)
    	print(st)
    	ang_vel = 1.5*(st - (3.1416/28))
    	if ang_vel>1.5:
    		ang_vel = 1.5
    	elif ang_vel <-1.5:
    		ang_vel = -1.5
    	else:
    		ang_vel = ang_vel
    	return ang_vel
    
    def get_loc(self, x_r1, y_r1, x_r2, y_r2, xt0, xt1, yt0, yt1, Eud2, Eud3):
    	
    	#### Initial Position
    	self.x_init = x_r1
    	self.y_init = y_r1
    	self.x_init1 = x_r2
    	self.y_init1 = y_r2
    	
    	##### Forward adjustment point 1 (Euclidean desitance 0)
    	"""
    	x_eu01 = x_r1[1]
    	y_eu01 = y_r1[1]
    	x_eu02 = x_r2[1]
    	y_eu02 = y_r2[1]
    	d0 = sqrt(pow(x_eu01 - x_eu02,2)+pow(y_eu01 - y_eu02,2))
	D0 = sqrt(pow(x_eu01,2)+pow(y_eu01,2))
	Del0 = sqrt(pow(x_eu02,2)+pow(y_eu02,2))
	alpha0 = acos((pow(d0,2)+pow(D0,2)-pow(Del0,2))/(2*D0*d0))
	xt0 = D0*cos(alpha0)
	yt0 = D0*sin(alpha0)
	"""
	 
    	#### Forward adjutment point 2 (Euclidean distance 1)
    	"""
    	x_eu11 = x_r1[2]
    	y_eu11 = y_r1[2]
    	x_eu12 = x_r2[2]
    	y_eu12 = y_r2[2]
 
    	d1 = sqrt(pow(x_eu11 - x_eu12,2)+pow(y_eu11 - y_eu12,2))
	D1 = sqrt(pow(x_eu11,2)+pow(y_eu11,2))
	Del1 = sqrt(pow(x_eu12,2)+pow(y_eu12,2))
	alpha1 = acos((pow(d1,2)+pow(D1,2)-pow(Del1,2))/(2*D1*d1))
	xt1 = D1*cos(alpha1)
	yt1 = D1*sin(alpha1)
    	"""
    	
    	#### Reverse adjutment point (Euclidean distance 2)
    	"""
    	x_eu21 = x_r1[3]
    	y_eu21 = y_r1[3]
    	x_eu22 = x_r2[3]
    	y_eu22 = y_r2[3]
    	Eud2 = sqrt(pow(x_eu21,2)+pow(y_eu21,2))
    	"""

    	#### Parking Point (Euclidean distance 3)
    	"""
    	x_eu31 = x_r1[4]
    	y_eu31 = y_r1[4]
    	x_eu32 = x_r2[4]
    	y_eu32 = y_r2[4]
    	Eud3 = x_eu32
    	"""
    	
    	vel_msg = Twist()
    	while self.euclidean_distance0(xt0, yt0) >= 0.05:
    		if self.stop_v:
    			vel_msg.linear.x = 0
    		else:
    			vel_msg.linear.x = self.linear_vel0()
    		vel_msg.linear.y = 0
    		vel_msg.linear.z = 0
    		vel_msg.angular.x = 0
    		vel_msg.angular.y = 0
    		vel_msg.angular.z = self.angular_vel0(xt0, yt0)
    		self.velocity_publisher.publish(vel_msg)
    		self.rate.sleep()
    		
    	while self.euclidean_distance1(xt1, yt1) >= 0.05:
    		if self.stop_v:
    			vel_msg.linear.x = 0
    		else:
    			vel_msg.linear.x = self.linear_vel1()
    		vel_msg.linear.y = 0
    		vel_msg.linear.z = 0
    		vel_msg.angular.x = 0
    		vel_msg.angular.y = 0
    		vel_msg.angular.z = self.angular_vel1(xt1, yt1)
    		self.velocity_publisher.publish(vel_msg)
    		self.rate.sleep()

    	while self.euclidean_distance2(Eud2) >= 0.05:
    		if self.stop_v:
    			vel_msg.linear.x = 0
    		else:
    			vel_msg.linear.x = self.linear_vel2()
    		vel_msg.linear.y = 0
    		vel_msg.linear.z = 0
    		vel_msg.angular.x = 0
    		vel_msg.angular.y = 0
    		vel_msg.angular.z = self.angular_vel2()
    		self.velocity_publisher.publish(vel_msg)
    		self.rate.sleep()
    		
    	while self.euclidean_distance3(Eud3) >= 0.04:
    		if self.stop_v:
    			vel_msg.linear.x = 0
    		else:
    			vel_msg.linear.x = self.linear_vel3()
    		vel_msg.linear.y = 0
    		vel_msg.linear.z = 0
    		vel_msg.angular.x = 0
    		vel_msg.angular.y = 0
    		vel_msg.angular.z = self.angular_vel3()
    		self.velocity_publisher.publish(vel_msg)
    		self.rate.sleep()
    	vel_msg.linear.x = 0
    	self.velocity_publisher.publish(vel_msg)
    	#Bus().get_message()
    	#self.lift_publisher.publish("UP")
    	#rospy.spin()
    	
if __name__ == '__main__':
    try:
    	reflector_detect().get_loc()
    except rospy.ROSInterruptException:
    	pass
