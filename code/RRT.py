import numpy as np 
import matplotlib.pyplot as plt
import matplotlib.patches as patches

import math
import random
from math import pi as pi
import heapq as hq

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

max_speed = 10#17.8816   #m/s
max_turn = 0.785398   #radians
unit_time = 0.2

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
    
    def draw(self, x,y):
        plt.plot([x, x+.01],[y, y +.01], '0.30',lw=0.75)
    
    def plot(self,node):
        plt.plot(node.x,node.y,marker='o', markersize=5, color="red")

    def collision_free(self, p1, p0=None):
        (x,y) = (p1.x, p1.y)
        #Check that point is inside bounds of the environment:
        if x < self.min_x or x > self.max_x or y < self.min_y or y > self.max_y:
            return False
    
        #Check that point isn't in a wall:
        point = Point([x,y])
        for o in self.obstacles:
            if point.within(o) and not point.touches(o):
                return False 
        return True

    #Check if a single point is collision free
    def sample_collision_free(self):
        x = random.uniform(self.min_x, self.max_x)
        y = random.uniform(self.min_y, self.max_y)
        theta = random.uniform(-pi, pi)
        p = node(x,y,theta)
        while not self.collision_free(p):
            x = random.uniform(self.min_x, self.max_x)
            y = random.uniform(self.min_y, self.max_y)
            theta = random.uniform(-pi, pi)
            p = node(x,y,theta)
        return p

    
		
############################ RRT Class ####################################
class node:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        self.lin_vel = 0
        self.ang_vel = 0
        self.children = []
        self.g=0
        self.parent=None
    def equal(self, node=None,pose=None):
        if node!=None:
            return self.x==node.x and self.y==node.y and self.theta==node.theta
        if pose!=None:
            return self.x==pose[0] and self.y==pose[1] and self.theta==pose[2]
    def set_parent(self,parent):
        self.parent=parent 
class RRT:
    nodes = []
    
    def build_tree(self, K, env, greedy=False):
        q0 = node(-9, -7.5, pi)
        self.nodes.append(q0)
        for i in range(K):
            print('Iteration ', i)
            q_rand = env.sample_collision_free()
            nnear = self.nearest_neighbor(q_rand)
            q_near = self.nodes[nnear]
            self.new_state(q_near, q_rand, env, greedy=False)

    #Returns index of nearest node
    def nearest_neighbor(self, q_rand):
        d_min = self.distance(self.nodes[0], q_rand)
        nnear = 0
        for i in range(1, len(self.nodes)):
            d = self.distance(self.nodes[i], q_rand)
            if d < d_min:
                nnear = i
        return nnear

    #Returns euclidean distance between two points
    def distance(self, n1, n2):
        euclidean = math.sqrt((n1.x - n2.x)**2+(n1.y - n2.y)**2)
        d_rot1 = math.atan2(n2.y-n1.y, n2.x-n1.x) - n1.theta
        d_rot2 = n2.theta - n1.theta - d_rot1
        return euclidean+d_rot1+d_rot2

    #Returns random linear and angular velocity controls
    def sample_random_control(self):
        l_vel = random.uniform(-1*max_speed, max_speed)
        ang_vel = random.uniform(-1*max_turn, max_turn)
        return (l_vel, ang_vel)
    
    #Samples multiple controls and propogates an edge from q_near given controls
    def new_state(self, q_near, q_rand, env, greedy=False):
        possible_controls = []
        for i in range(10):
            possible_controls.append(self.sample_random_control())

        trajectories = []
        for control in possible_controls:
            trajectories.append(self.trajectory(q_near, control))

        if greedy == True:
            goal=node(10, 6.5, pi)
            #Find control closest to random node
            dmin = self.distance(trajectories[0][-1], goal) / 2
            nnear = 0
            for i in range(1, len(trajectories)):
                n = trajectories[i][-1]
                d = self.distance(n, q_rand) / 2 
                if d < dmin:
                    nnear = i
        else:
            #Find control closest to random node
            dmin = self.distance(trajectories[0][-1], q_rand)
            nnear = 0
            for i in range(1, len(trajectories)):
                n = trajectories[i][-1]
                d = self.distance(n, q_rand)
                if d < dmin:
                    nnear = i
                
        #Check for collisions:
        collision = False
        for point in trajectories[nnear]:
            if not env.collision_free(point):   #Doesn't take into account too sharp turns
                collision = True
                break
        
        if collision == False:
            q_new = trajectories[nnear][-1] 
            for n in trajectories[nnear]:
                env.draw(n.x,n.y)
            self.nodes.append(q_new)                                    #Add node to our tree
            q_near.children.append((q_new, possible_controls[nnear]))   #Append child with control necessary to reach that node

    #Return list of x,y,theta nodes generated taking a certain curve
    def trajectory(self, n, control):
        path = []
        d0 = node(n.x, n.y, n.theta)
        path.append(d0)
        speed = control[0]
        turn = control[1]
        dt = 0.05

        #Integrate control forwards:
        for i in range(1,int(unit_time/dt)):
            theta = path[i-1].theta+speed*math.tan(turn)*dt
            x = path[i-1].x+speed*math.cos(path[i-1].theta)*dt
            y = path[i-1].y+speed*math.sin(path[i-1].theta)*dt    
            di = node(x,y, theta)
            path.append(di)
        return path
    def get_goal_index(self):
        goal=node(10, 6.5, pi)
        shortest=100
        index=0
        for i in range(len(self.nodes)):
            if self.distance(self.nodes[i],goal)<shortest:
                index=i
        return index
    def get_neighbor(self,check):
        neighbors=[]
        for node in self.nodes:
            for child in check.children:
                if node.equal(child[0]):
                    neighbors.append(node)
        return neighbors
                
######################### A Star ##################################################

def find_path(start, goal, T):
    fringe = [] #Nodes we are considering expanding
    closed = [] #Nodes we have visited
    #get start spot
    start.g = 0 #Starting node has g value = 0
    startnode=start
    #Put start in the fringe with f value = 0:
    hq.heappush(fringe, (0,startnode))
    current_tuple=None
    counter = 0
    
    while(fringe):
        #Pop node with the smallest f value from heap
        current_tuple = hq.heappop(fringe)
        
        counter+=1
            
        closed.append(current_tuple[1])
        
        if current_tuple[1].equal(goal):
            print ("Found Goal!")
            print("step %d"%(counter))
            goal.set_parent(current_tuple[1].parent)
            goal.g=current_tuple[1].g
            return True
        
        #get neighbors list
        neighbors = T.get_neighbor(current_tuple[1])
        
        for neighbor in neighbors:
            in_closed =False
            for traversed in closed:
                if traversed.equal(neighbor):
                    in_closed = True
                
            if not in_closed:
                in_open = False
                for open_node in fringe:
                    if open_node[1].equal(neighbor):
                        in_open = True
                        neighbor=open_node[1]
                if not in_open:
                    neighbor.g=-1
                    neighbor.set_parent(None)
                    
                updateVertex(current_tuple[1],neighbor,fringe,goal)
    print ("No path found")
    return False        
def heuristic(n1, n2):
    euclidean = math.sqrt((n1.x - n2.x)**2+(n1.y - n2.y)**2)
    d_rot1 = math.atan2(n2.y-n1.y, n2.x-n1.x) - n1.theta
    d_rot2 = n2.theta - n1.theta - d_rot1
    return euclidean+d_rot1+d_rot2
#    dx = abs(n.x - m.x)
#    dy = abs(n.y - m.y)
#    h_n = np.sqrt(math.pow(dx,2) + math.pow(dy,2))
##    h_n=dx+dy
#    return h_n 
def travel(n1,n2):
    for child in n1.children:
        if n2.equal(child[0]):
            return child[1][0]*unit_time
def updateVertex(current,succ,fringe,goal):
    #use heuristic to get cost of travel (assume heuristic=cost)
    c_val=travel(current,succ)
    #update g value for vertex
    if current.g + c_val < succ.g or succ.g<0:
        succ.g = current.g+c_val
        succ.set_parent(current)
        for open_node in fringe:
            if open_node[1].equal(succ):
                fringe.remove(open_node)    
        h_n = heuristic(succ,goal)
        f_n = h_n + succ.g 
        hq.heappush(fringe, (f_n,succ))

###########################################################################  
def average(intlist):
    total=0
    for val in intlist:
        total+=val
    if len(intlist)>0:
        return total/float(len(intlist))
    else:
        return -1
def sd(intlist):
    avg= average(intlist)
    variance=0
    for val in intlist:
        variance+=np.square(val-avg)
    if len(intlist)>0:
        variance= variance/float(len(intlist))
        return np.sqrt(variance)
    else:
        return -1

def main():
    m = Map(-9, 10, -7.5, 6.5)
    m.add_obstacle((6,2.9), (6.3,2.9), (6.3,-4.2), (6,-4.2))
    m.add_obstacle((1.2,6.5), (1.5,6.5),  (1.5,-1.5), (1.2,-1.5))
    m.add_obstacle((-4.2,1), (-4.2,-7.5), (-4.5,1), (-4.5,-7.5))
    T = RRT()
    T.build_tree(200, m, greedy=True)
    ''' 
    stats=[]
    for i in range(5):
        raw_dat=[]
        for j in range(5):
            T = RRT()
            T.build_tree(i*20+200, m, greedy=False)
            start=T.nodes[0]
            goal=T.nodes[T.get_goal_index()]
            find_path(start,goal,T)
            raw_dat.append(goal.g+T.distance(goal,node(10,6.5,pi))*1.5)
        stats.append((average(raw_dat),sd(raw_dat)))
    print (stats)
    start=T.nodes[0]
    goal=T.nodes[T.get_goal_index()]
    find_path(start,goal,T)
    iterator=goal
    while iterator!=None:
        m.plot(iterator)
        iterator=iterator.parent
    '''
    plt.show()
if __name__ == "__main__":
    main()
