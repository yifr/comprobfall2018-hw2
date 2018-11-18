# rrt.py
# This program generates a simple rapidly
# exploring random tree (RRT) in a rectangular region.

import sys, random, math, pygame
from pygame.locals import *
from math import sqrt,cos,sin,atan2


class node():
    pose = ()   #Tuple contains (x,y,theta) values
    twist = ()  #Twist contains linear and angular velocities

    def __init__(self, pose=(0,0,math.pi), twist=(0,0)):
        self.pose = pose
        self.twist = twist

#constants
pi = math.pi
XDIM = 640
YDIM = 480
WINSIZE = [XDIM, YDIM]
EPSILON = 7.0
NUMNODES = 5000

#Euclidean distance between two points
def dist(p1,p2):
    return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))


def step_from_to(p1,p2):
    if dist(p1,p2) < EPSILON:
        return p2
    else:
        theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
        return p1[0] + EPSILON*cos(theta), p1[1] + EPSILON*sin(theta)

def random_state():
    x = random.random()*640.0
    y = random.random()*480.0
    theta = random.random()*360.0
    return (x,y)


def reachable_from(start, goal):
    theta1 = start.pose[2]
    theta2 = goal.pose[2]
    if math.abs(theta1 - theta2) <= 45:
        return True
    else:
        return False

def main():
    #initialize and prepare screen
    pygame.init()
    screen = pygame.display.set_mode(WINSIZE)
    white = 255, 240, 200
    black = 20, 20, 40
    screen.fill(black)
    
    #Define start and stop locations
    START = (0, 480)   #Start on bottom left 
    STOP = (640, 0, 0)
    
    nodes = []

    nodes.append(START) 

    for i in range(NUMNODES):
        #Sample collision free point
        rand = random_state() 
        '''
        while not collision_free(rand):
            rand = random_state
        '''
        n_neighbor = nodes[0]    
        
        #Connect new node to nearest neighbor
        for p in nodes:
            #  if reachable_from(p, rand):
            if  dist(p,rand) < dist(n_neighbor,rand):
                n_neighbor = p
        
  #      u = get_transition()

        newnode = step_from_to(n_neighbor,rand)
        nodes.append(newnode)
        pygame.draw.line(screen,white,n_neighbor,newnode)
        pygame.display.update()
        #print i, "    ", nodes

        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Leaving because you requested it.")
    

# if python says run, then we should run
if __name__ == '__main__':
    main()


