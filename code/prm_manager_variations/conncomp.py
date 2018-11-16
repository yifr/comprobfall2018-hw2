from math import e


def prm_cc(graph,start,goal,iterations):
    """Samples and connects new points"""
    for i in range(iterations):
        graph.sample()
        nb=graph.ngd(graph.vertices[i])
        for pt in nb:
            graph.connect(graph.vertices[i],pt)
    """Add in start and goal locations to the end"""
    graph.addVertex(start)
    nb=graph.ngd(graph.vertices[-1])
    for pt in nb:
        graph.connect(graph.vertices[-1],pt)
    graph.addVertex(goal)
    nb=graph.ngd(graph.vertices[-1])
    for pt in nb:
        graph.connect(graph.vertices[-1],pt)
        
def prm_k(graph,start,goal,iterations,k):
    """Samples and connects new points"""
    for i in range(iterations):
        graph.sample()
        nb=graph.ngd(graph.vertices[i])
        for j in range(k):
            pt = nb[j]
            graph.connect(graph.vertices[i],pt)
    """Add in start and goal locations to the end"""
    graph.addVertex(start)
    nb=graph.ngd(graph.vertices[-1])
    for j in range(k):
        pt = nb[j]
        graph.connect(graph.vertices[i],pt)
    graph.addVertex(goal)
    nb=graph.ngd(graph.vertices[-1])
    for j in range(k):
        pt = nb[j]
        graph.connect(graph.vertices[i],pt)



def prm_star():
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

roadmapbuild()
#astar_search()