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
from Astar import find_path
import PRM

STEP=0.5

class Graph():
    def __init__(self,mp):
        self.edges=[]
        self.vertices=[]
        self.mp=mp
    def addVertex(self,point):
        self.vertices.append(point)
    def copy(self):
        temp=Graph(self.mp)
        temp.edges=self.edges
        temp.vertices=self.vertices
        return temp
    def ngd(self, a):
        neighbors=[]
        distances=[]
        for pt in self.vertices:
            if pt!=a:
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
            quat_list=quat.rotation_matrix
            flat_quat=[]
            for lis in quat_list:
                for val in lis:
                    flat_quat.append(val)
            valid= str(pqp_client(pose,flat_quat))=="result: False"
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
        		#print temp_pose,temp_orient
        		result=str(pqp_client(temp_pose,flat_quat))=="result: False"
        		#print result
        		collision_free=result
		
        		i+=1
        return not collision_free
    
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
                if node1==None and nodes[counter].equal(pose=edge[0]):
                    node1=nodes[counter]
                if node2==None and nodes[counter].equal(pose=edge[1]):
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
    def __init__(self,pose):
        self.pose=pose
        self.connected=[]
        self.parent = None
    
    def equal(self,node=None,pose=None):
        if node!=None:
            if self.pose==node.pose:
                return True
            else:
                return False
        elif pose!=None:
            if self.pose==pose:
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
    a=n[0]
    b=m[0]
    return np.sqrt(np.square(a[0]-b[0])+np.square(a[1]-b[1])+np.square(a[2]-b[2]))
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
def test():
    #	pointa=((3,4,2),(1.0,0.0,0.0,0.0))
    #	pointb=((3,6,2),(0,2.0,1.0,4.0))
    #	pm=pianoMover()	
    #	pm.move_model("piano2",pointb[0],pointb[1])
    #
    #	mp=XYZmap(-10,10,-10,10,-10,10)
    #	quat=Quaternion(pointa[1])
    #	quat_list=quat.rotation_matrix
    #	flat_quat=[]
    #	for lis in quat_list:
    #		for val in lis:
    #		    flat_quat.append(val)
    #
    #	print (pqp_client(pointa[0],flat_quat)==False)
    #	print mp.collide(pointa,pointb)
    pm=pianoMover()	
    pm.move_model("piano2",(3.5,4,0.4),(0,0,0,0))
def q5():
    mp = XYZmap(-10,10,-10,10,0.4,5)
    starts=[]
    goals=[]
    for i in range(20):
        starts.append(mp.sample())
        goals.append(mp.sample())
    prmcc_maps=[]
    prmk_maps=[]
    prmstar_maps=[]
    for i in range(10):
        prmcc_maps.append(PRM.prm_cc(Graph(mp),50*i))
        prmk_maps.append(PRM.prm_k(Graph(mp),50*i,5))
        prmstar_maps.append(PRM.prm_star(Graph(mp),50*i))
    cc_stats=[]
    k_stats=[]
    star_stats=[]
    for prm_map in prmcc_maps:
        raw_dat=[]
        for i in range(len(starts)):
            temp=prm_map.copy()
            PRM.prm_cc(temp,0,starts[i],goals[i])
            ag=interpret(temp.vertices,temp.edges)
            end = ag.nodes.pop()
            beg = ag.nodes.pop()
            if find_path(beg,end):
                raw_dat.append(end.g)
        cc_stats.append(average(raw_dat),sd(raw_dat))
    for prm_map in prmk_maps:
        raw_dat=[]
        for i in range(len(starts)):
            temp=prm_map.copy()
            PRM.prm_k(temp,0,starts[i],goals[i])
            ag=interpret(temp.vertices,temp.edges)
            end = ag.nodes.pop()
            beg = ag.nodes.pop()
            if find_path(beg,end):
                raw_dat.append(end.g)
        k_stats.append(average(raw_dat),sd(raw_dat))
    for prm_map in prmstar_maps:
        raw_dat=[]
        for i in range(len(starts)):
            temp=prm_map.copy()
            PRM.prm_star(temp,0,starts[i],goals[i])
            ag=interpret(temp.vertices,temp.edges)
            end = ag.nodes.pop()
            beg = ag.nodes.pop()
            if find_path(beg,end):
                raw_dat.append(end.g)
        star_stats.append(average(raw_dat),sd(raw_dat))
    print cc_stats
    print k_stats
    print star_stats
def main():

	mp = XYZmap(-10,10,-10,10,0.4,5)

	start=((3.0,8.5,0.4),(0.0,0.0,0.0,0.0))
	goal=((3.5,4.0,0.4),(0.0,0.0,0.0,0.0))

	graph=Graph(mp)
	

	#    PRM.prm_cc(graph,100,start,goal)
	PRM.prm_k(graph,50,3,start,goal)
	#    PRM.prm_star(graph,100,start,goal)
	
	

	ag=interpret(graph.vertices,graph.edges)
	end = ag.nodes.pop()
	beg = ag.nodes.pop()
	find_path(beg,end)
	iterator = end
	pts=[]
	print graph.edges
	while iterator.parent != None:
		pts.append(iterator.pose)
		iterator=iterator.parent
	pts.append(iterator.pose)
	pm=pianoMover()	
	print pts
	pts.reverse()
	print pts
	latter=pts[0]
	for pt in pts:
		iterations=int(mp.distance(latter,pt)/STEP)
		i=0
		while i < iterations:
			print latter[0][1],",",pt[0][1],",",i
			temp_pose=(latter[0][0]+(pt[0][0]-latter[0][0])*i/float(iterations),
				   latter[0][1]+(pt[0][1]-latter[0][1])*i/float(iterations),
				   latter[0][2]+(pt[0][2]-latter[0][2])*i/float(iterations))
			temp_orient=(latter[1][0]+(pt[1][0]-latter[1][0])*i/float(iterations),
				     latter[1][1]+(pt[1][1]-latter[1][1])*i/float(iterations),
				     latter[1][2]+(pt[1][2]-latter[1][2])*i/float(iterations),
				     latter[1][3]+(pt[1][3]-latter[1][3])*i/float(iterations))
			pm.move_model("piano2",temp_pose,temp_orient)
			print temp_pose,temp_orient
			i+=1
		pm.move_model("piano2",pt[0],pt[1])
		latter=pt
		#pm.move_model("piano2",pt[0],pt[1])
	
        #pm.move_model("piano2",end.pose[0],end.pose[1])
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
