def roadmapbuild():
#    G.init() not sure about how to intialize map with start and end goals
    i = 0
    while i < 50:
        a = sample()
        if collision(a) == true: #not sure if i'm passing correct parameters for collision
            G.addVertex(a)
            i = i + 1
        k = 10 #can change k value
        nb = ngd(a) #nb is list of neighbors of a
        nb = nb[0:k] #returns first k of list
        for q in nb:
            if connect(a,q):
#               G.addEdge(a,q)

#def astar_search()

roadmapbuild()
#astar_search()