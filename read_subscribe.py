#!/usr/bin/env python

import rospy
import numpy as np
import time
import copy


from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist



 # Initialize the node



class my_map(object):
	
	def __init__ (self):
		rospy.Subscriber("/map",OccupancyGrid, self.map)
		self.data = None
		self.pose_x = None
		self.pose_y = None
		self.resolution = None
		self.width = None
		self.height = None

	def map(self, data):
		self.data = data.data
		self.pose_x = data.info.origin.position.x
		self.pose_y = data.info.origin.position.y
		self.resolution = data.info.resolution
		self.width = data.info.width
		self.height = data.info.height

def map_normalize():
 	msg = copy.deepcopy(mapa)
	print('it got the data')
	xy = np.array(msg.data, dtype=np.float64)

	for i in range(0,len(xy)):

		if xy[i]==-1.0:
			xy[i]=0.4	
		elif xy[i]==100:
			xy[i]=0
		elif xy[i]==0:
			xy[i]=1
		else:
			xy[i]=0.7

	map_norm=(xy.reshape(msg.height,msg.width))

	return (map_norm)


def extract_map(map_norm):
	print('you entered to extract')
	#Posicion del mapa respecto al eje de coordenadas. Se obvia la orientacion por ser la misma
	pose_map_x = odom_data.x - mapa.pose_x
	pose_map_y = odom_data.y - mapa.pose_y
	# Posicion del robot en el mapa
 	index_map_n = int(pose_map_x/mapa.resolution)
 	index_map_m = int(pose_map_y/mapa.resolution)

 	#print('X', pose_map_x, ' Y: ', pose_map_y)

 	#print('index pose: n :', index_map_n,'  m:  ', index_map_m)	


 	#Ancho de porcion de mapa
 	delta = 40
 	# Extraer una porcion del mapa
 	ext_map = map_norm[(index_map_m - delta):(index_map_m+delta),(index_map_n-delta):(index_map_n+delta)]

	#print(ext_map)
	#fig, ax = plt.subplots()
	#plt.gray(
	#plt.imshow(ext_map)
	#plt.show()

	#print('odom.x: ', odom_data.x, 'mapa.x: ', mapa.pose_x)
	#print('odom.y: ', odom_data.y, 'mapa.y: ', mapa.pose_y)

 	#print('le', pose_map_x_d, ' y: ', pose_map_y_d)
 	#print('index pose: n :', index_map_n,'  m:  ', index_map_m)	
	
 	index_ext_m_d = 0
	index_ext_n_d = 0

	cond_filtered = 0



	# They all go through filters
	while cond_filtered == 0:
		#print('WHAT')
		#random index desired
		"""
		mayor_cond = 0
		while mayor_cond == 0:
			random = np.random.randn(2,1)
			random = np.absolute(random) - 1
			random = -random
			if all(i > 0 for i in random) == True:
				#print("error 5")
				mayor_cond = 1

		"""




		index_ext_m_d = int(2*delta*(np.random.rand()))
		index_ext_n_d = int(2*delta*(np.random.rand()))

		#print("error 1")
		if ext_map[index_ext_m_d,index_ext_n_d] == 1 or  ext_map[index_ext_m_d,index_ext_n_d] == 0.4:
			yd = index_ext_m_d
			xd = index_ext_n_d
			x0 = delta
			y0 = delta
			r = 12

			th = np.arctan2(yd-y0,xd-x0)
			"""
			x1 = x0 - r*np.sin(th) - r*np.cos(th)
			y1 = y0 + r*np.cos(th) - r*np.sin(th)

			x2 = x0 + r*np.sin(th) - r*np.cos(th)
			y2 = y0 - r*np.cos(th) - r*np.sin(th)

			x3 = xd + r*np.cos(th) + r*np.sin(th)
			y3 = yd + r*np.sin(th) - r*np.cos(th)

			x4 = xd + r*np.cos(th) - r*np.sin(th)
			y4 = yd + r*np.sin(th) + r*np.cos(th)
			"""



			x1 = x0 + r*np.cos(th+2.3562);
			y1 = y0 + r*np.sin(th+2.3562);

			x2 = x0 + r*np.cos(th-2.3562);
			y2 = y0 + r*np.sin(th-2.3562);

			x3 = xd + r*np.cos(th-0.7854);
			y3 = yd + r*np.sin(th-0.7854);

			x4 = xd + r*np.cos(th+0.7854);
			y4 = yd + r*np.sin(th+0.7854);


			recta_1 = [(y1 - y2)/(x1*y2 - x2*y1),-(x1 - x2)/(x1*y2 - x2*y1),1]
			recta_2 = [(y2 - y3)/(x2*y3 - x3*y2),-(x2 - x3)/(x2*y3 - x3*y2),1]
			recta_3 = [(y3 - y4)/(x3*y4 - x4*y3),-(x3 - x4)/(x3*y4 - x4*y3),1]
			recta_4 = [(y4 - y1)/(x4*y1 - x1*y4),-(x4 - x1)/(x4*y1 - x1*y4),1]

			numpy_extract = np.array(ext_map)
			to_append_in_region = []
			(m_h,n_w) = numpy_extract.shape
			map_of_the_region = np.zeros((m_h,n_w))
			#print ("Error 2")
			#Definen regiones
			a1 = recta_1 [0]
			b1 = recta_1 [1]
			c1 = recta_1 [2]
			if (a1*delta+b1*delta+c1) > 0:
				
				recta_1 = [ -x for x in recta_1]
				a1 = recta_1 [0]
				b1 = recta_1 [1]
				c1 = recta_1 [2]

			a2 = recta_2 [0]
			b2 = recta_2 [1]
			c2 = recta_2 [2]
			if (a2*delta+b2*delta+c2) > 0:
				
				recta_2 = [ -x for x in recta_2]
				a2 = recta_2 [0]
				b2 = recta_2 [1]
				c2 = recta_2 [2]

			a3 = recta_3 [0]
			b3 = recta_3 [1]
			c3 = recta_3 [2]
			if (a3*delta+b3*delta+c3) > 0:
				
				recta_3 = [ -x for x in recta_3]	
				a3 = recta_3 [0]
				b3 = recta_3 [1]
				c3 = recta_3 [2]

			a4 = recta_4 [0]
			b4 = recta_4 [1]
			c4 = recta_4 [2]

			if (a4*delta+b4*delta+c4) > 0:
				
				recta_4 = [ -x for x in recta_4]
				a4 = recta_4 [0]
				b4 = recta_4 [1]
				c4 = recta_4 [2]	
			print('recta_1: ', recta_1,'recta_2: ', recta_2)
			#print ("Error 3")
			#Iteracion para todo el mapa
			print(n_w,'  ',m_h)
			for i in range(0,n_w-1):
				for j in range (0,m_h-1):
					if a1*i+b1*j+c1 < 0 :
						#print("error 4_1")
						if a2*i+b2*j+c2 < 0:
							#print("error 4_2")
							if a3*i+b3*j+c3 < 0:
								#print("error 4_3")
								if a4*i+b4*j+c4 < 0:
									#print("error 4_4")
									to_append_in_region.append(ext_map[j,i])
									map_of_the_region[j,i] = ext_map[j,i]

			print ('region: ',to_append_in_region)
			if all(i >= 0.2 for i in to_append_in_region) == True:
				#print("error 5")
				cond_filtered = 1
			print("index no valido")

	#fig, ax = plt.subplots()
	#plt.gray()
	#plt.imshow(map_of_the_region)
	#plt.show()

	index_map_n_d = index_map_n + (index_ext_n_d-delta)
	index_map_m_d = index_map_m + (index_ext_m_d-delta)

	## COnversion to the pose from frame

	pose_map_y_d = index_map_m_d*mapa.resolution + mapa.pose_y
	pose_map_x_d = index_map_n_d*mapa.resolution + mapa.pose_x

	print('odom.x: ', odom_data.x, 'mapa.x: ', mapa.pose_x)
	print('odom.y: ', odom_data.y, 'mapa.y: ', mapa.pose_y)

 	#print('le', pose_map_x_d, ' y: ', pose_map_y_d)
 	print('i am: n :', index_map_n,'  m:  ', index_map_m)
 	print('index desired n: ', index_map_n_d,'  index desired m:', index_map_m_d)
 	print('pose map_x_d: ', pose_map_x, 'pose map_y_d: ', pose_map_y_d)
 	print('it is perfect')



	#fig, ax = plt.subplots()
	#plt.gray()
	#plt.imshow(map_norm)
	#plt.show()

 	return(pose_map_x_d,pose_map_y_d)

"""

def random_selection(ext_map):
	index_map_x_d = 0
	index_map_y_d = 0
	cond_filtered = 0
	# They all go through filters
	while cond_filtered == 0:
		#print('WHAT')
		index_map_x_d = int(50*(np.random.rand()))
		index_map_y_d = int(50*(np.random.rand()))
		if ext_map[index_map_x_d,index_map_y_d] == 0:
			xd = index_map_x_d
			yd = index_map_y_d
			x0 = 25
			y0 = 25

			rho_1
			rho_2
			rho_3
			rho_4
			if m:

				cond_filtered = 1



	
	return(index_map_x_d, index_map_y_d)

"""




"""
def get_map():
	msg = rospy.wait_for_message("/map", OccupancyGrid)
	print('it got the data')
	xy = np.array(msg.data, dtype=np.float64)

	for i in range(0,len(xy)):

		if xy[i]==-1.0:
			xy[i]=0.4	
		elif xy[i]==100:
			xy[i]=0
		elif xy[i]==0:
			xy[i]=1
		else:
			xy[i]=0.4

	map_norm=xy.reshape(msg.info.height,msg.info.width)
	x_m = msg.info.origin.position.x
	y_m = msg.info.origin.position.y
	res = msg.info.resolution
	
	return (map_norm, x_m, y_m, res)

def extract_map(map_norm, x_m, y_m, res):
	print('you entered to extract')
	pose_map_x = odom_data.x - x_m
	pose_map_y = odom_data.y - y_m

 	index_map_x = int(pose_map_x/res)
 	index_map_y = int(pose_map_y/res)

 	#ext_map = [index_map_x,index_map_y]
 	ext_map = map_norm[(index_map_x-50):(index_map_x+50),(index_map_y-50):(index_map_y+50)]
 	print('le',ext_map)
 	print('it is perfect')
 	return ext_map
""" 	


#vuelta de 360

## extract map (map, pose)

## extract a portion of the map

## select randomly a desired pose

## filters, filter 1
"""

def filtering (extrmap, xipose, yipose, xides, yides)
	
	if np.extrmap[xides,yides] == 0
		error_d = 1
		return 1


"""
class Subs_data_odom(object):


	def __init__ (self):

		rospy.Subscriber("/gazebo/model_states", ModelStates, self.odometry)
		


		self.x = 0
		self.y = 0

		self.q_z = 0
		self.q_w = 0

		self.th = 0

		self.ddot = 0
		self.thdot = 0
		
		self.u_dis = 0
		self.u_angl = 0


		self.e_angl = 0
		self.e_dist = 0
		
		self.e_angle_dot = 0
		self.e_dist_dot = 0

	def odometry (self, data):
		self.x = data.pose[2].position.x
		self.y = data.pose[2].position.y
		
		self.q_z = data.pose[2].orientation.z
		self.q_w = data.pose[2].orientation.w

		angle_denorm = 2*np.arctan2(self.q_z,self.q_w)-np.pi
		
		if self.q_z >= 0:
			if angle_denorm > np.pi:
				self.th = angle_denorm -  2*np.pi 
			else:
				self.th = angle_denorm
		else:
			if angle_denorm < -np.pi:
				self.th = angle_denorm +  2*np.pi 
			else:
				self.th = angle_denorm
		
		self.ddot = data.twist[2].linear.x
		self.thdot = data.twist[2].angular.z




	def pid_controller(self,x_d,y_d,x_no,y_no,kpl,sign_dis):

		 self.e_dist = np.sqrt((y_d-self.y)**2+(x_d-self.x)**2)

		 th_d = np.arctan2(y_d-y_no,x_d-x_no)

		 self.e_dist_dot = -self.ddot

		 self.e_angl = th_d - self.th
		 self.e_angle_dot = self.thdot

		 # Definir parametros
		 kdp =-0.15
		 kap =0.25*kpl
		 kdd = 0.15
		 kad =0.15*kpl



		 self.u_dis = sign_dis*((kdp*self.e_dist) + (kdd*self.e_dist_dot))
		 if self.u_dis > 1:
		 	self.u_dis = 1
		 self.u_angl = kap*self.e_angl + kad*self.e_angle_dot
		 if self.e_angle_dot > 1:
		 	self.u_angl = kap*self.e_angl + kad*1
		 if self.u_angl >1:
		 	self.u_angl = 1
		 if sign_dis == -1:

		 	self.u_angl =0



def pubvel (x_d,y_d):
	 time.sleep(1)
	 x_no = copy.copy(odom_data.x)
	 y_no = copy.copy(odom_data.y)
	 motion = Twist()
	 odom_data.pid_controller(x_d,y_d,x_no,y_no,1,+1)
	 

	 print(x_no,y_no)
	 time.sleep(1)

	 print('it is on')
	 while abs(odom_data.e_angl) > 0.01 or abs(odom_data.e_angle_dot)>0.01:

	 	 odom_data.pid_controller(x_d,y_d,x_no,y_no,1,+1)
	 	 #print('angle: ', odom_data.th, 'err_a: ', odom_data.e_angl)
		 motion.angular.z = odom_data.u_angl

		 pub_vel.publish(motion)
		 
		 #print('e_ang:',odom_data.e_angl)

     

	 motion.angular.z = 0.0
	 sign = np.sign(x_d-odom_data.x)
	 
	 while abs(odom_data.e_dist) > 0.1 or abs(odom_data.e_dist_dot)>0.1:
	 	 #odom_data.pid_controller(x_d,y_d,x_no,y_no,1,+1)
	 	 #print('dist: ', odom_data.e_dist)
	 	 #print('angle: ', odom_data.th, 'err_a: ', odom_data.e_angl)
	 	 #print('salidas : ', odom_data.u_dis, ' ', odom_data.u_angl )
	 	 
	 	 sign_change = np.sign(x_d-odom_data.x)
	 	 #print('dist: ', odom_data.e_dist)

	 	 if sign == sign_change:
	 	 	odom_data.pid_controller(x_d,y_d,x_no,y_no,2,+1)
	 	 else:
	 	 	odom_data.pid_controller(x_d,y_d,x_no,y_no,0.6,-1)

		 

	 	 motion.linear.x = odom_data.u_dis
	 	 motion.angular.z = odom_data.u_angl
	 	 #print('salida',odom_data.u_dis)
		 #print(motion)
	 	 pub_vel.publish(motion)
	 	 #print('e_dist: ', odom_data.e_dist, 'e_dis_dot:' , odom_data.e_dist_dot, '   ', odom_data.ddot )
	 	 
	 motion.linear.x = 0.0
	 motion.angular.z = 0.0

def pubvel_round (x_d,y_d):
	 time.sleep(0.5)
	 x_no = copy.copy(odom_data.x)
	 y_no = copy.copy(odom_data.y)
	 motion = Twist()
	 odom_data.pid_controller(x_d,y_d,x_no,y_no,1,+1)
	 

	 print(x_no,y_no)
	 time.sleep(0.5)

	 print('it is on')
	 while abs(odom_data.e_angl) > 0.1 or abs(odom_data.e_angle_dot)>0.05:

	 	 odom_data.pid_controller(x_d,y_d,x_no,y_no,1.5,+1)
	 	 #print('angle: ', odom_data.th, 'err_a: ', odom_data.e_angl)
		 motion.angular.z = odom_data.u_angl

		 pub_vel.publish(motion)
		 
		 #print('e_ang:',odom_data.e_angl)

     

	 motion.angular.z = 0.0

   	 

if __name__ == '__main__':

	rospy.init_node('read_subscribe')
		
	pub_vel = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size =10)
	time.sleep(1)
	#rospy.Subscriber("map", OccupancyGrid, Mapa.data)
	mapa = my_map()
	odom_data = Subs_data_odom()
	time.sleep(1)
	iteration = 0



	while 

	mapa_normalizado = map_normalize()
	fig, ax = plt.subplots()
	plt.gray()
	plt.imshow(mapa_normalizado)
	plt.show()



	
	"""
	fig, ax = plt.subplots()
	plt.gray()
	plt.imshow(mapa_normalizado)
	plt.show()
	print(mapa_extraido)

	fig, ax = plt.subplots()
	plt.gray()
	plt.imshow(mapa_extraido)
	plt.show()
	"""

	#index_map_desired=random_selection(mapa_extraido)

	print('you got it')

	



