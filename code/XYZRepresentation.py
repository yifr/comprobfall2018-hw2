# -*- coding: utf-8 -*-
"""
Created on Sat Nov 17 14:43:56 2018

@author: lawrence
"""
from pyquaternion import Quaternion
import random as rand
import numpy as np
from pqp_ros_client import pqp_client
from Piano_Mover import pianoMover
import math

STEP=0.5

class Graph():
    def __init__(self,mp):
        self.edges=[]
        self.vertices=[]
        self.mp=mp
    def addVertex(self,point):
        self.vertices.append(point)
    def ngd(self, a):
        neighbors=[]
        distances=[]
        for pt in self.vertices:
            if pt==a:
                dist=self.mp.distance(pt,a)
                if not distances:
                    distances.append(dist)
                    neighbors.append(pt)
                else:
                    counter=0
                    while counter<len(distances) and dist>distances[counter]:
                        counter+=1
                    distances.insert(counter,dist)
                    neighbors.insert(counter,pt)
        return neighbors
    def can_connect(self,a,q):
        return not self.mp.collide(a,q)
    def connect(self,a,q):
        if self.can_connect(a,q):
            if (a,q) not in self.edges and (q,a) not in self.edges:
                self.edges.append((a,q))
    def sample(self):
        samp=self.mp.sample()
        self.addVertex(samp)
        return samp
    
class XYZmap():
    def __init__(self,lowx,highx,lowy,highy,lowz,highz):
        self.lx=lowx
        self.hx=highx
        self.ly=lowy
        self.hy=highy
        self.lz=lowz
        self.hz=highz
    def sample(self):
        valid=False
        while not valid:
            pose=(rand.uniform(self.lx,self.hx),rand.uniform(self.ly,self.hy),rand.uniform(self.lz,self.hz))
            orientation=(rand.uniform(-1,1),rand.uniform(-1,1),rand.uniform(-1,1),rand.uniform(-1,1))
            quat = Quaternion(orientation)
            valid= not pqp_client(pose,quat.rotation_matrix)
        return (pose,orientation)
    def distance(self,pointa,pointb):
        a=pointa[0]
        b=pointb[0]
        return np.sqrt(np.square(a[0]-b[0])+np.square(a[1]-b[1])+np.square(a[2]-b[2]))
    def collide(self,pointa,pointb):
        iterations=int(self.distance(pointa,pointb)/STEP)
        collision_free=True
        i=0
        while collision_free and i < iterations:
            temp_pose=(pointa[0][0]+(pointb[0][0]-pointa[0][0])*i/float(iterations),
			   pointa[0][1]+(pointb[0][1]-pointa[0][1])*i/float(iterations),
			   pointa[0][2]+(pointb[0][2]-pointa[0][2])*i/float(iterations))
		temp_orient=(pointa[1][0]+(pointb[1][0]-pointa[1][0])*i/float(iterations),
			     pointa[1][1]+(pointb[1][1]-pointa[1][1])*i/float(iterations),
			     pointa[1][2]+(pointb[1][2]-pointa[1][2])*i/float(iterations),
			     pointa[1][3]+(pointb[1][3]-pointa[1][3])*i/float(iterations))
            quat=Quaternion(temp_orient)
            quat_list=quat.rotation_matrix
            flat_quat=[]
            for lis in quat_list:
                for val in lis:
                    flat_quat.append(val)
            collision_free=not pqp_client(temp_pose,flat_quat)
            i+=1
        return collision_free
    
def interpret(vertices,edges):
    graph=aStarGraph(vertices,edges)
    return graph
        
class aStarGraph():
    def __init__(self,vertices,edges):
        #initialize nodes and add connections
        nodes=[]
        for vertex in vertices:
            nodes.append(node(vertex))
        #add edges
        for edge in edges:
            node1=None
            node2=None
            counter=0
            while node1==None or node2==None:
                if node1==None and nodes[counter].equal(coord=edge[0]):
                    node1=nodes[counter]
                if node2==None and nodes[counter].equal(coord=edge[1]):
                    node2=nodes[counter]
                counter+=1
            node1.connected.append(node2)
            node2.connected.append(node1)
        self.nodes=nodes
    def get_adjacent(self,x,y):
        for current_vert in self.vertices:
            if current_vert.x==x and current_vert.y==y:
                return current_vert.connected

class node():
    g=0
    vis=False
    def __init__(self,coord):
        self.x=coord[0]
        self.y=coord[1]
        self.connected=[]
        self.parent = None
    
    def equal(self,node=None,coord=None):
        if node!=None:
            if self.x==node.x and self.y==node.y:
                return True
            else:
                return False
        elif coord!=None:
            if self.x==coord[0] and self.y==coord[1]:
                return True
            else:
                return False
        else:
            return False
    def set_parent(self,parent):
        self.parent=parent 
        
    def store_node(self,store):
        self.connected.append(store)

    #Using straight line distance as heuristic
def heuristic(n, m):
    dx = abs(n.x - m.x)
    dy = abs(n.y - m.y)
    h_n = np.sqrt(math.pow(dx,2) + math.pow(dy,2))
#    h_n=dx+dy
    return h_n 
def main():
    pointa=((0,0,1),(1.0,0.0,0.0,0.0))
    pointb=((10,0,1),(0,2.0,1.0,4.0))
	
    mp=XYZmap(-10,10,-10,10,-10,10)
    pm=pianoMover()
    print pm.collide(pointa,pointb)
    
#	
#	iterations=int(mp.distance(pointa,pointb)/STEP)
#	i=0
#	while i < iterations:
#		temp_pose=(pointa[0][0]+(pointb[0][0]-pointa[0][0])*i/float(iterations),
#			   pointa[0][1]+(pointb[0][1]-pointa[0][1])*i/float(iterations),
#			   pointa[0][2]+(pointb[0][2]-pointa[0][2])*i/float(iterations))
#		temp_orient=(pointa[1][0]+(pointb[1][0]-pointa[1][0])*i/float(iterations),
#			     pointa[1][1]+(pointb[1][1]-pointa[1][1])*i/float(iterations),
#			     pointa[1][2]+(pointb[1][2]-pointa[1][2])*i/float(iterations),
#			     pointa[1][3]+(pointb[1][3]-pointa[1][3])*i/float(iterations))
#		pm.move_model("piano2",temp_pose,temp_orient)
#		print temp_orient
#		i+=1
if __name__ == "__main__":
    	main()