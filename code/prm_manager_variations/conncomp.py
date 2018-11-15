def roadmapbuild():
#    G.init() not sure about how to intialize map and dimensions with start and end goals
    i = 0
    while i < 50 #50 iterations
        a = sample()
        if collision(a) == true: #not sure if i'm passing correct parameters for collision
            G.addVertex(a)
            i = i + 1
            l = []
            if len(ngd(a)) == 0:
                a_l = list(a)
                l = l.append(a_l)
            else:
                for q in ngd(a):
                    connbool = False
                    for i 0:len(l)
                        if q in l[i] & a not in l[i]:
                            connbool = True
                    if connect(a, q) & connbool:
                        #G.addEdge(a,q)
                        for i 0:len(l):
                            if q in l[i]:
                                q = l.pop(l[i])
                                q = q.append(a)
                                l = q.append(l)
                            else:
                                a_l2 = list(a)
                                l = l.append(a)

#def astar_search()

roadmapbuild()
#astar_search()