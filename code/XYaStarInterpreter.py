# -*- coding: utf-8 -*-
"""
Created on Tue Nov 13 15:50:30 2018

@author: lawrence
"""


import numpy as np

import math

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
                elif node2==None and nodes[counter].equal(coord=edge[1]):
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

def test():
    vg = interpret([(0,0),(0,1),(0,2)],[((0,0),(0,1)),((0,2),(0,0))])
    for node in vg.nodes:
        print "(",node.x,",",node.y,")"," [",
        for nd in node.connected:
            print "(",nd.x,",",nd.y,")",
        print "]"
        
if __name__ == '__main__':
    test()