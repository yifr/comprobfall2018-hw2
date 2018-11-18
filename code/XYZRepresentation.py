# -*- coding: utf-8 -*-
"""
Created on Sat Nov 17 14:43:56 2018

@author: lawrence
"""
from pyquaternion import Quaternion
import random as rand
import numpy as np
from pqp_ros_client import pqp_client
from Piano_Mover import move_model

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
            temp_pose=(pointa[0][0]+(pointb[0][0]-pointa[0][0])*i/iterations,
                       pointa[0][1]+(pointb[0][1]-pointa[0][1])*i/iterations,
                       pointa[0][2]+(pointb[0][2]-pointa[0][2])*i/iterations)
            temp_orient=(pointa[1][0]+(pointb[1][0]-pointa[1][0])*i/iterations,
                         pointa[1][1]+(pointb[1][1]-pointa[1][1])*i/iterations,
                         pointa[1][2]+(pointb[1][2]-pointa[1][2])*i/iterations,
                         pointa[1][3]+(pointb[1][3]-pointa[1][3])*i/iterations)
            quat=Quaternion(temp_orient)
            collision_free=not pqp_client(temp_pose,quat.rotation_matrix)
            i+=1
        return collision_free
def main():
    pointa=((0,0,10)(0,0,0,0))
    pointb=((10,0,10)(0,0,0,0))
    mp=XYZmap(0,10,0,10,0,10)
    iterations=int(mp.distance(pointa,pointb)/STEP)
    i=0
    while i < iterations:
        temp_pose=(pointa[0][0]+(pointb[0][0]-pointa[0][0])*i/iterations,
                   pointa[0][1]+(pointb[0][1]-pointa[0][1])*i/iterations,
                   pointa[0][2]+(pointb[0][2]-pointa[0][2])*i/iterations)
        temp_orient=(pointa[1][0]+(pointb[1][0]-pointa[1][0])*i/iterations,
                     pointa[1][1]+(pointb[1][1]-pointa[1][1])*i/iterations,
                     pointa[1][2]+(pointb[1][2]-pointa[1][2])*i/iterations,
                     pointa[1][3]+(pointb[1][3]-pointa[1][3])*i/iterations)
        move_model("piano2",temp_pose,temp_orient)
        i+=1
if __name__ == "__main__":
    main()