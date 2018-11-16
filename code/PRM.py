
import math

def prm_cc(graph,start,goal,iterations):
    """Samples and connects new points"""
    conn_comp=[]
    for i in range(iterations+2):
        """Add sampe or start and goal locations to the end"""
        conn_comp.append([])
        if i<iterations:
            samp=graph.sample()
        elif i==iterations:
            graph.addVertex(start)
            samp=start
        else:
            graph.addVertex(goal)
            samp=goal
        conn_comp[-1].append(samp)
        """Get and connect neighbors"""
        nb=graph.ngd(graph.vertices[i])
        for pt in nb:
            connectable=False
            if graph.can_connect(pt,samp):
                connectable=True
                c=0
                pt_group=-1
                samp_group=-1
                while c<len(conn_comp):
                    group = conn_comp[c]
                    if pt in group:
                        pt_group=c
                    if samp in group:
                        samp_group=c
                    if samp in group and pt in group:
                        connectable=False
                    c+=1
            if connectable and samp_group >=0 and pt_group>=0:
                conn_comp[pt_group]=conn_comp[pt_group]+conn_comp[samp_group]
                conn_comp.pop(samp_group)
                graph.connect(graph.vertices[i],pt)
        
def prm_k(graph,start,goal,iterations,k):
    """Samples and connects new points"""
    for i in range(iterations+2):
        if i<iterations:
            graph.sample()
        elif i==iterations:
            graph.addVertex(start)
        else:
            graph.addVertex(goal)
        """Connect nearest k"""
        nb=graph.ngd(graph.vertices[i])
        nb_counter=k
        if len(nb)<k:
            nb_counter=len(nb)
        for j in range(nb_counter):
            pt = nb[j]
            graph.connect(graph.vertices[i],pt)


def prm_star(graph,start,goal,iterations):
    for i in range(iterations+2):
        """Add sampe or start and goal locations to the end"""
        if i<iterations:
            samp=graph.sample()
        elif i==iterations:
            graph.addVertex(start)
            samp=start
        else:
            graph.addVertex(goal)
            samp=goal
        """Calculate number of neighbors to add"""
        d = 2 #since this is 2d representation, switch to 6d for gazebo
        k = int(math.e*((1/d) + 1)*(math.log(len(graph.vertices)))+1) #can change k value and round up
        """Add neighbors"""
        nb = graph.ngd(samp) #nb is list of neighbors of a
        nb = nb[0:k] #returns first k of list
        for q in nb:
            graph.connect(samp,q)
        graph.mp.points=graph.vertices
        graph.mp.edges=graph.edges
        graph.mp.display()

def test():
    print int(10.9)

if __name__ == '__main__':
    test()
