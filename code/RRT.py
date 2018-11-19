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

    def __init__(self, x1, x2, y1, y2):
        self.min_x = x1
        self.max_x = x2
        self.min_y = y1
        self.max_y = y2

    #Shapely library used for all collision detection
    def add_obstacle(self, p1, p2, p3, p4):
        point1 = Point(p1)        
        point2 = Point(p2)
        point3 = Point(p3)
        point4 = Point(p4)

        polygon = Polygon([point1, point2, point3, point4])
        self.obstacles.append(polygon)
        

    def collision_free(self, p_new, p_old):
        (x,y) = (p_new[0], p_new[1][2])
        if x < self.min_x or x > self.max_x or y < self.min_y or y > self.max_y:
            return False
         
        point = Point([x,y])
        for o in self.obstacles:
            if point in o:
                return False
        
        return True

    def sample_collision_free(self, T, n):
        x = random.uniform(T.min_x, T.max_x)
        y = random.uniform(T.min_y, T.max_y)
        theta = random.uniform(-pi, pi)
        p = (n,x,y,theta)
        while not T.collision_free(p):
            x = random.uniform(T.min_x, T.max_x)
            y = random.uniform(T.min_y, T.min_y)
            theta = random.uniform(-pi, pi)
            p = (n,x,y,theta)
    
        return p


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
        
        #self.pose = (x,y,theta)

        self.parent = []
        self.parent.append(0)

        self.lin_vel = 0
        self.ang_vel = 0
        
        #self.twist = (self.lin_vel, self. ang_vel)
    
    def distance(self, p1, p2):
        euclidean = math.sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))
        return euclidean

    
    def sample_random_twist(self):
        lin_vel = random.uniform(1, 17.8816)
        sign = random.uniform(0,1)
        if sign < .5:   #randomly decelerate?
            lin_vel *= -1
        ang_vel = random.uniform(-1*self.max_turn, self.max_turn)
        
        return (lin_vel, ang_vel)
    
    #Propogates edge from nnear given nrand using randomly sampled controls
    def step_from_to(self, T, nnear, nrand):
        (xnear, ynear, theta_near) = (self.x[nnear], self.y[nnear], self.theta[nnear])
        (xrand,yrand) = (self.x[nrand], self.y[nrand])
        
        #Lists to hold reachable states 
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

            if not T.collision_free(p1, p0):
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

    def nearest_neighbor(self, n):
        d_min = self.distance(0, n)
        nnear = 0
        for i in range(1, n):
            d = self.distance(i, n)
            if d < d_min:
                nnear = i
        return nnear

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

    def expand(self, T):
        n = len(self.x)    #Index number for new node
        (x,y,theta) = T.sample_collision_free() #Sample random point from state space
        self.add_node(n, x, y, theta)
        nnear = self.nearest_neighbor(n)
        self.step_from_to(T, nnear, n)
        
        
###########################################################################  


##################### Construct Map in Matplotlib #########################
fig = plt.figure()
ax = plt.axes(xlim = (10, -9), ylim = (-7.5, 6.5))

wall_1 = patches.Rectangle((6,-4.2),.3, 7.1, fill=True)
wall_2 = patches.Rectangle((1.2, -1.5), width=.3, height=7.5, fill=True)
wall_3 = patches.Rectangle((-4.5, -7.5), width=.3, height=8.5, fill=True)

ax.add_patch(wall_1)
ax.add_patch(wall_2)
ax.add_patch(wall_3)
plt.show()
###########################################################################
