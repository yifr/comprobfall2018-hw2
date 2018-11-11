# -*- coding: utf-8 -*-
"""
Created on Mon Nov  5 15:06:34 2018

@author: lawrence
"""

import numpy as np
import random as rd

import matplotlib.pyplot as plt
import matplotlib.path as mplpath
import matplotlib.patches as patches
from matplotlib.collections import PatchCollection
from matplotlib import collections as mc
import shapely.geometry as shapely
#from descartes.patch import PolygonPatch

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
            if pt[0]!=a[0] or pt[1]!=a[1]:
                dist=np.sqrt(np.square(pt[0]-a[0])+np.square(pt[1]-a[1]))
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
    def connect(self,a,q):
        if self.mp.collide(a,q):
            return False
        else:
            if (a,q) not in self.edges and (q,a) not in self.edges:
                self.edges.append((a,q))
            return True
class Map():
    points=[]
    edges=[]
    lines=[]
    buff_lines=[]
    obstacles=[]
    buff=0
    vis_graph=None
    visibility=False
    def __init__(self,lx=0,hx=0,ly=0,hy=0):
        self.lx=lx
        self.hx=hx
        self.ly=ly
        self.hy=hy
    
    def sample(self):
        x=None
        y=None
        while x==None or self.collide((x,y)):
            xwidth=self.hx-self.lx
            ywidth=self.hy-self.ly
            x=xwidth*rd.random()+self.lx
            y=ywidth*rd.random()+self.ly
        return (x,y)
        
    def collide(self,point1,point2=None):
        if point2==None:
            point2=point1
        path =shapely.LineString([point1,point2])
        if self.visibility:
            for obs in self.obstacles:
                if obs[4].intersects(path) and not path.touches(obs[4]):
                    return True
        else:
            if self.buff>0:
                path=path.buffer(self.buff)
            for obs in self.obstacles:
                if obs[1].intersects(path) and not path.touches(obs[1]):
                    return True
        return False
        
    def clear(self):
        self.lines=[]
        self.buff_lines=[]
        self.obstacles=[]
        
    #vertices are of format [(a,b),(c,d)]
    def add_Poly(self,vertices=[]):
        if self.visibility:
            init_poly=shapely.Polygon(vertices)
            buff_poly=init_poly.buffer(self.buff,cap_style=2,join_style =2)
            new_vert=list(buff_poly.exterior.coords)
            self.obstacles.append((mplpath.Path(vertices),init_poly,new_vert,mplpath.Path(new_vert),buff_poly))
        else:
            self.obstacles.append((mplpath.Path(vertices),shapely.Polygon(vertices),vertices))
        
    def display(self):
        #start grid
        fig, ax = plt.subplots()
    
        #Add Obstacles
        if self.buff>0 and self.visibility:
            for poly in self.obstacles:
                ax.add_patch(patches.PathPatch(poly[3], facecolor='orange',alpha=0.5))
        for poly in self.obstacles:
            ax.add_patch(patches.PathPatch(poly[0], facecolor='purple'))
        
        #Set boundaries
        ax.set_xlim([self.lx,self.hx])
        ax.set_ylim([self.ly,self.hy])
        
        #add edges
#        vis_lines=[]
        c=[]
        for edge in self.edges:
            c.append((1,0,0,0.5))
#            vis_lines.append([(edge[0][0],edge[0][1]),(edge[1][0],edge[1][1])])
        ax.add_collection(mc.LineCollection(self.edges,colors=c,linewidths = 2.5))
        
        #Set size of plot
        fig_size = plt.rcParams["figure.figsize"]
        fig_size[0] = 12
        fig_size[1] = 9
        plt.rcParams["figure.figsize"] = fig_size
        
        #draw points
        for pt in self.points:
            plt.plot(pt[0],pt[1],marker='o', markersize=5, color="red")
        
        #draw lines to the plot
        if not self.visibility and self.buff>0:
            path=shapely.LineString(self.buff_lines)
            poly=path.buffer(self.buff)
            poly_points=list(poly.exterior.coords)
            ax.add_patch(patches.PathPatch(mplpath.Path(poly_points), facecolor='yellow',alpha=0.5))
        lc = mc.LineCollection(self.lines,linewidths = 2.5)
        ax.add_collection(lc)
        
        
        #Set up Axis 
        ax.minorticks_on()
        ax.xaxis.set_major_locator(plt.MultipleLocator(1))
        ax.yaxis.set_major_locator(plt.MultipleLocator(1))
        ax.xaxis.set_minor_locator(plt.MultipleLocator(0.25))
        ax.yaxis.set_minor_locator(plt.MultipleLocator(0.25))
        ax.grid(which='major', linestyle='-', linewidth='0.5', color='black')
        ax.grid(which='minor', linestyle='-', linewidth='0.2', color='black')
    
        plt.show()

def test():
    mp = Map(-5,5,-5,5)
    mp.add_Poly([(-1,-1),(-1,1),(1,1),(1,-1)])
    graph=Graph(mp)
#    for i in range(10):
#        pt=mp.sample()
#        graph.addVertex(pt)
#        pts.append(pt)
    for i in range(100):
        graph.addVertex(mp.sample())
        nb=graph.ngd(graph.vertices[i])
        for pt in nb:
            graph.connect(graph.vertices[i],pt)
        mp.points=graph.vertices
        mp.edges=graph.edges
        mp.display()
#    graph.addVertex((-2,-2))
#    print graph.ngd(graph.vertices[0])
#    graph.addVertex((2,2))
#    print graph.ngd(graph.vertices[0])
#    graph.addVertex((-2,2))
#    nb=graph.ngd(graph.vertices[0])
#    for pt in nb:
#        graph.connect(graph.vertices[0],pt)
#    print graph.edges
#    mp.edges=[((2,2),(-2,2))]
    mp.points=graph.vertices
    mp.edges=graph.edges
    mp.display()
#    print map.collide((1,1))

    
if __name__ == '__main__':
    test()