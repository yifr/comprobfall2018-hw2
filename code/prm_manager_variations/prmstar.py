from math import e


def roadmapbuild():
#    G.init() not sure about how to intialize map with start and end goals
    i = 0
    while i < 50:
        a = sample()
        if collision(a) == true: #not sure if i'm passing correct parameters for collision
            G.addVertex(a)
            i = i + 1
        nb = ngd(a) #nb is list of neighbors of a
        d = 2 #since this is 2d representation, switch to 6d for gazebo
        a = e
        b = (1/d) + 1
        c = math.log(len(nb) + 1)
        k = round(a*b*c) #can change k value
        nb = nb[0:k] #returns first k of list
        for q in nb:
            if connect(a,q):
#               G.addEdge(a,q)

#def astar_search()
def test():
    roadmapbuild()
#astar_search()
if __name__ == '__main__':
    test()