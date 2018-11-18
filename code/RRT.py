import math
from math import pi as pi
import numpy as np 
import matplotlib.patches as patches
import matplotlib.pyplot as plt
from matplotlib import animation

########################### Construct Map ################################
fig = plt.figure()
ax = plt.axes(xlim = (10, -9), ylim = (-7.5, 6.5))

wall_1 = patches.Rectangle((6,-4.2),.3, 7.1, fill=True)
wall_2 = patches.Rectangle((1.2, -1.5), width=.3, height=7.5, fill=True)
wall_3 = patches.Rectangle((-4.5, -7.5), width=.3, height=8.5, fill=True)

ax.add_patch(wall_1)
ax.add_patch(wall_2)
ax.add_patch(wall_3)
###########################################################################


############################## RRT Class #################################
class RRT:
    tire_dia = 0.14605                          #Tired diameter
    max_speed = 17.8816                         #Max speed (m/s)
    max_ang = 2*pi*(max_speed/(pi*tire_dia))    #Max angular velocity (rad/s)
    max_turn = 45                               #+/- 45 degrees for max turn
    unit_time = 1                               #1 sec interval unit time 
     
    def __init__(self, x, y, theta):
        self.x = []
        self.x.append[x]

        self.y = []
        self.y.append(y)

        self.theta = []
        self.theta.append(theta)

        self.parent = []
        self.parent.append(0)

        self.lin_vel = 0
        self.ang_vel = 0
          
    
    #K is +/- 1 for accelerating or reversing
    def linear_velocity(K):
        return K*max_speed
    
    #alpha is a trajectory parameter 
    def angular_velocity(K, alpha):
        return K*alpha/pi*max_ang
    
    def distance(self, n1, n2):
        euclidean = sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))
        rotation = atan2(p2[1]-p1[1],p2[0]-p1[0])
        return euclidean + rotation

    def step(self, nnear, nrand):
        (xnear, ynear, theta_near) = (self.x[nnear], self.y[nnear], self.theta[nnear])
        (xrand,yrand,theta_rand) = (self.x[nrand], self.y[nrand], self.theta[nrand])
        
        #Lists to hold reachble states 
        x_r = []
        y_r = []
        theta_r = []

        #Sample 10 random controls
        for i in range(10):
            (speed, angle) = sample_random_twist()
            (x,y,theta) = self.trajectory(xnear, ynear, theta_near, angle, speed)
            x_r.append(x)
            y_r.append(y)
            theta_r.append(theta)

        #find nearest reachable state from near to rand
        d_min = ((xrand - x_r[0][-1])**2 + (yrand-y_r[0][-1])**2)**0.5
        near = 0
        for i in range(1, len(x_r)):
            d = ((xrand - x_r[0][-1])**2 + (yrand-y_r[0][-1])**2)**0.5
            if d < d_min:
                d_min = d
                near = i

       self.remove_node(nrand)

       #Check for collisions
       collision = False
       for i in range(0, len(x_r[near])):
           if is_free(x_r[near][i], y_r[near][i]):
               collision = True
               break
       if collision == False:
           self.add_node(nrand, x_r[near][-1], y_r[near][-1], theta_r[near][-1])
           self.add_edge(nnear, nrand, x_r[near], y_r[near])


    #Calculates result of moving from x_near with control z
    def trajectory(self, x_i, y_i, theta_i, angle, speed):
        (x,y,theta) = ([], [], [])
        x.append(x_i)
        y.append(y_i)
        theta.append(theta_i)
        dt = 0.01 
        
        #Integrate controls forward
        for i in range(1, int(self.unit_time/dt)):
            theta.append(theta[i-1]+speed*math.tan(angle)/1.9*dt)
            x.append(x[i-1]+speed*math.cos(theta[i-1]))
            y.append(y[i-1]+speed*math.sin(sin(theta[i-1])*dt))
        
        return (x,y,theta)
    
    def expand(self):
        nrand = sample_collision_free()
        nnear = self.near(n)
        self.step_from_to(nnear, n)     #Adds new node / edge to tree

        self.add_node(n, x, y, theta)
        
