import numpy as np 
import matplotlib.pyplot as plt
import matplotlib.patches as patches

import math
import random
from math import pi as pi

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon


############################ Map Class ####################################
class Map:
	obstacles = []
	fig = plt.figure()
	ax = plt.axes()

	def __init__(self, x1, x2, y1, y2):
		self.min_x = x1
		self.max_x = x2
		self.min_y = y1
		self.max_y = y2
		self.ax = plt.axes(xlim=(x1, x2), ylim=(y1, y2))

	#Shapely library used for all collision detection
	def add_obstacle(self, p1, p2, p3, p4):
		pointList = []
		pointList.append(Point(p1))
		pointList.append(Point(p2))
		pointList.append(Point(p3)) 
		pointList.append(Point(p4)) 
		
		polygon = Polygon([[p.x, p.y] for p in pointList])
		self.obstacles.append(polygon)

		width = abs(p3[0] - p1[0])
		height = abs(p3[1] - p2[1])
		wall = patches.Rectangle(p4,width,height, fill=True)
		self.ax.add_patch(wall)
  
	def display(self):
		plt.show()

	def collision_free(self, p_new, p_old=None):
		(x,y) = (p_new[1], p_new[2])
		if x < self.min_x or x > self.max_x or y < self.min_y or y > self.max_y:
			return False
		
		if p_old != None:
			print(p_old)
			theta_old = p_old[2]
			theta_new = p_new[2]
			if theta_new > theta_old + 0.785398 or theta_new < theta_old - 0.785398:
				print("Turning too sharply")
				return False
		 
		point = Point([x,y])
		for o in self.obstacles:
			if o.contains(point):
				return False
		return True
	
	def sample_collision_free(self, n):
		x = random.uniform(self.min_x, self.max_x)
		y = random.uniform(self.min_y, self.max_y)
		theta = random.uniform(-pi, pi)
		p = (n,x,y,theta)
		while not self.collision_free(p):
			x = random.uniform(self.min_x, self.max_x)
			y = random.uniform(self.min_y, self.min_y)
			theta = random.uniform(-pi, pi)
			p = (n,x,y,theta)

		return p

	#Arbitrary definition of being within goal region
	def in_goal(self, node, goal):
		if goal[0] - node[0] < 2 and goal[1] - node[1] < 2:
			return True

############################ RRT Class ####################################
class RRT:
	tire_dia = 0.14605                          #Tire diameter
	max_speed = 17.8816                         #Max speed (m/s)
	#max_ang = 2*pi*(max_speed/(pi*tire_dia))    #Max angular velocity (rad/s)
	max_turn = 0.785398                         #+/- 45 degrees for max turn (0.78.. radians)
	 
	def __init__(self, x, y, theta):
		self.x = []
		self.edge_x = []
		self.x.append(x)
		self.edge_x.append(x)
		self.y = []
		self.edge_y = []
		self.y.append(y)
		self.edge_y.append(y)

		self.theta = []
		self.theta.append(theta)
		
		self.parent = []
		self.parent.append(0)

		self.lin_vel = 0
		self.ang_vel = 0
			
	def expand(self, M):
		
		n = len(self.x)    #Index number for new random node
		(n,x,y,theta) = M.sample_collision_free(n)  #Sample random collision free pose from state space 
		self.add_node(n, x, y, theta)               #Add it to map (temporarily)
		nnear = self.nearest_neighbor(n)            #Find nearest neighbor
		self.step_from_to(M, nnear, n)              #Propogate control from that node
	
	def distance(self, n1, n2):
		p1 = (self.x[n1], self.y[n1])
		p2 = (self.x[n2], self.y[n2])
		euclidean = math.sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))
		return euclidean
	
	def nearest_neighbor(self, n):
		d_min = self.distance(0, n)
		nnear = 0
		for i in range(1, n):
			d = self.distance(i, n)
			if d < d_min:
				nnear = i
		return nnear

	def sample_random_twist(self):
		lin_vel = random.uniform(1, self.max_speed)
		sign = random.uniform(0,1.01)
		if sign < .5:   #randomly decelerate?
			lin_vel *= -1
		ang_vel = random.uniform(-1*self.max_turn, self.max_turn)
		
		#print('Sampling Random Twist: ', lin_vel, ang_vel)
		return (lin_vel, ang_vel)
	
	#Propogates edge from nnear given nrand using randomly sampled controls
	def step_from_to(self, M, nnear, nrand):
		(xnear, ynear, theta_near) = (self.x[nnear], self.y[nnear], self.theta[nnear])
		(xrand,yrand) = (self.x[nrand], self.y[nrand])
		
		#Lists to hold potentially reachable states 
		x_r = []
		y_r = []
		theta_r = []

		#1) Sample 10 random controls
		for i in range(10):
			(speed, angle) = self.sample_random_twist()
			(x,y,theta) = self.calculate_trajectory(xnear, ynear, theta_near, angle, speed) #returns lists of points along trajectory
			x_r.append(x)
			y_r.append(y)
			theta_r.append(theta)

		#2) find nearest reachable state from near to rand
		d_min = ((xrand - x_r[0][-1])**2 + (yrand-y_r[0][-1])**2)**0.5
		near = 0
		for i in range(1, len(x_r)):
			d = ((xrand - x_r[0][-1])**2 + (yrand-y_r[0][-1])**2)**0.5
			if d < d_min:
				d_min = d
				near = i

		self.remove_node(nrand)

	   #Check for collisions along trajectory
		collision = False
		for i in range(0, len(x_r[near])):
			p1 = (x_r[near][i], y_r[near][i], theta_r[near][i])
			p0 = (xnear, ynear, theta_near)

			if not M.collision_free(p1, p0):
				collision = True
				break

		if collision == False:
			self.add_node(nrand, x_r[near][-1], y_r[near][-1], theta_r[near][-1])
			self.add_edge(nnear, nrand, x_r[near], y_r[near])


	#Calculates result of moving from x_near with randomly sampled control
	def calculate_trajectory(self, x_i, y_i, theta_i, angle, speed):
		(x,y,theta) = ([], [], [])
		x.append(x_i)
		y.append(y_i)
		theta.append(theta_i)
		dt = 0.01 
		unit_time = random.uniform(0.05, 1)     #Randomly sample time?

		#Integrate controls forward
		for i in range(1, int(unit_time/dt)):
			x.append(x[i-1]+speed*math.cos(theta[i-1])*dt)
			y.append(y[i-1]+speed*math.sin(theta[i-1])*dt)
			theta.append(theta[i-1]+speed*math.tan(angle)*dt)

		return (x,y,theta)

	def add_node(self, n, x, y, theta):
		self.x.insert(n, x)
		self.y.insert(n, y)
		self.theta.insert(n, theta)

	def remove_node(self, n):
		self.x.pop(n)
		self.y.pop(n)
		self.theta.pop(n)

	def add_edge(self, parent, child, x, y):
		self.parent.insert(child, parent)
		self.edge_x.append(x)
		self.edge_y.append(y)


	def drawTree(self):			
		for i in range (1,len(self.x)):
			plt.plot(self.edge_x[i],self.edge_y[i],'0.25',lw=0.5)
		
###########################################################################  


def main():
	m = Map(-9, 10, -7.5, 6.5)
	m.add_obstacle((6,2.9), (6.3,2.9), (6.3,-4.2), (6,-4.2))
	m.add_obstacle((1.2,6.5), (1.5,6.5),  (1.5,-1.5), (1.2,-1.5))
	m.add_obstacle((-4.2,1), (-4.2,-7.5), (-4.5,1), (-4.5,-7.5))
	#m.display()
	
	G = RRT(-9, -7.5, pi)
	for i in range(500):
		G.expand(m)
	G.drawTree()

	m.display()

if __name__ == "__main__":
	main()