# -*- coding: utf-8 -*-
"""
Created on Tue Nov 13 14:04:58 2018

@author: lawrence
"""

import numpy as np

import math

import heapq as hq
    
######################
# A* implementation  # 
######################
def find_path(start, goal):
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
            print "Found Goal!"
            print("step %d"%(counter))
            goal.set_parent(current_tuple[1].parent)
            goal.g=current_tuple[1].g
            return True
        
        #get neighbors list
        neighbors = current_tuple[1].connected
        
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
    print "No path found"
    return False        
def heuristic(n, m):
#    a=n.pose[0]
#    b=m.pose[0]
#    return np.sqrt(np.square(a[0]-b[0])+np.square(a[1]-b[1])+np.square(a[2]-b[2]))
    dx = abs(n.x - m.x)
    dy = abs(n.y - m.y)
    h_n = np.sqrt(math.pow(dx,2) + math.pow(dy,2))
#    h_n=dx+dy
    return h_n 
def updateVertex(current,succ,fringe,goal):
    #use heuristic to get cost of travel (assume heuristic=cost)
    c_val=heuristic(current,succ)
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
    
