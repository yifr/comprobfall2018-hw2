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
def find_path(start, goal,vis=None):
    fringe = [] #Nodes we are considering expanding
    closed = [] #Nodes we have visited
    start.g = 0 #Starting node has g value = 0
    startnode=start
    #get start spot
    startnode=vis.vertices[0]
    #Put start in the fringe with f value = 0:
    hq.heappush(fringe, (0,startnode))
    current_tuple=None
    counter = 0
    
    while(fringe):
        #Pop node with the smallest f value from heap
        current_tuple = hq.heappop(fringe)
        
        counter+=1
            
        closed.append(current_tuple[1])
        
        if current_tuple[1].x == goal.x and current_tuple[1].y == goal.y:
                print "Found Goal!"
                print("step %d"%(counter))
                goal.set_parent(current_tuple[1].parent)
                goal.g=current_tuple[1].g
                return closed
        
        #get neighbors list
        neighbors = current_tuple[1].connected
        
        for neighbor in neighbors:
            in_closed =False
            for traversed in closed:
                if traversed.x==neighbor.x and traversed.y==neighbor.y:
                    in_closed = True
                
            if not in_closed:
                in_open = False
                for open_node in fringe:
                    if open_node[1].x==neighbor.x and open_node[1].y==neighbor.y:
                        in_open = True
                        neighbor=open_node[1]
                if not in_open:
                    neighbor.g=-1   
                    neighbor.set_parent(None)
                    
                updateVertex(current_tuple[1],neighbor,fringe,goal)
               
    return "No path found"

def updateVertex(current,succ,fringe,goal):
    difx=succ.x-current.x
    dify=succ.y-current.y
    c_val= np.sqrt(math.pow(difx,2)+math.pow(dify,2))
#        print ("g: %f, f: %f, curr_g:%f"% (current.g + c_val,current.g + c_val+heuristic(succ,goal),succ.g))
    if current.g + c_val < succ.g or succ.g<0:
        succ.g = current.g+c_val
        succ.set_parent(current)
        for open_node in fringe:
            if open_node[1].x==succ.x and open_node[1].y==succ.y:
                fringe.remove(open_node)    
        h_n = heuristic(succ,goal)
        f_n = 1.0*h_n + 1.0*succ.g 
#            print ("f: %f"% (f_n))
#            print "wooh"
        hq.heappush(fringe, (f_n,succ))
    