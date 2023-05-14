# Standard Algorithm Implementation
# Sampling-based Algorithms PRM

import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
import random
from scipy import spatial


# Class for PRM
class PRM:
    # Constructor
    def __init__(self, map_array):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.samples = []                     # list of sampled points
        self.graph = nx.Graph()               # constructed graph
        self.path = []                        # list of nodes of the found path


    def check_collision(self, p1, p2):
        '''Check if the path between two points collide with obstacles
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            True if there are obstacles between two points
        '''
        ### YOUR CODE HERE ###
        points = []
        x,y = p1
        dx = abs(p2[0] - p1[0])
        dy = abs(p2[1] - p1[1])
        s1,s2 = 0,0
        interchange = 0
        if(p2[0] - p1[0] < 0):
            s1 = -1
        elif(p2[0] - p1[0] > 0):
            s1 = 1
        if(p2[1] - p1[1] < 0):
            s2 = -1
        elif(p2[1] - p1[1] > 0):
            s2 = 1
        if(dy > dx):
            temp = dx
            dx = dy
            dy = temp
            interchange = 1
        e = 2*dy - dx
        a = 2*dy
        b = 2*dy - 2*dx
        points.append([x,y])
        for i in range(dx):
            if(e<0):
                if(interchange == 1):
                    y = y + s2
                else:
                    x = x + s1
                e = e + a
            else:
                y = y + s2
                x = x + s1
                e = e + b
            points.append([x,y])
        check = False
        for point in points:
            if self.map_array[point[0]][point[1]] == 0:
                check = True
                break
        return check


    def dis(self, point1, point2):
        '''Calculate the euclidean distance between two points
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            euclidean distance between two points
        '''
        ### YOUR CODE HERE ###
        distance = np.sqrt(np.square(point1[0] - point2[0]) + np.square(point1[1] - point2[1]))
        return distance


    def uniform_sample(self, n_pts):
        '''Use uniform sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()
        ### YOUR CODE HERE ###
        maxX,maxY = len(self.map_array), len(self.map_array[0])
        xS = np.linspace(0,maxX-1,num=int(np.sqrt(n_pts)),dtype=np.int32)
        yS = np.linspace(0,maxY-1,num=int(np.sqrt(n_pts)),dtype=np.int32)
        for x in xS:
            for y in yS:
                if(self.map_array[x][y] == 1):
                    self.samples.append((x,y))
        #self.samples.append((0, 0))


    
    def random_sample(self, n_pts):
        '''Use random sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        maxX,maxY = len(self.map_array), len(self.map_array[0])
        for i in range(n_pts):
            x = random.randint(0,maxX-1)
            y = random.randint(0,maxY-1)
            if(self.map_array[x][y] == 1):
                self.samples.append((x,y))
        #self.samples.append((0, 0))


    def gaussian_sample(self, n_pts):
        '''Use gaussian sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        maxX,maxY = len(self.map_array), len(self.map_array[0])
        for i in range(n_pts):
            x = random.randint(0,maxX-1)
            y = random.randint(0,maxY-1)
            new = np.random.normal([x,y],10,size=(1,2))
            newX = int(new[0][0])
            newY = int(new[0][1])
            if(newX<0 or newX>=maxX or newY<0 or newY>=maxY):
                continue
            if(self.map_array[x][y] == 1 and self.map_array[newX][newY] == 1):
                #do nothing
                nothing = 0
            elif(self.map_array[x][y] == 0 and self.map_array[newX][newY] == 0):
                #do nothing
                nothing = 0
            else:
                if(self.map_array[x][y] == 1):
                    self.samples.append((x,y))
                else:
                    self.samples.append((newX,newY))
        #self.samples.append((0, 0))


    def bridge_sample(self, n_pts):
        '''Use bridge sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        maxX,maxY = len(self.map_array), len(self.map_array[0])
        for i in range(n_pts):
            q1 = None
            while(True):
                x = random.randint(0,maxX-1)
                y = random.randint(0,maxY-1)
                q1 = [x,y]
                if(self.map_array[q1[0]][q1[1]] == 0):
                    break
            q2 = np.random.normal([q1[0],q1[1]],20,size=(1,2))
            q2 = [int(q2[0][0]),int(q2[0][1])]
            if(q2[0]<0 or q2[0]>=maxX or q2[1]<0 or q2[1]>=maxY):
                continue
            if(self.map_array[q2[0]][q2[1]] == 0):
                midX = q1[0] + int((q2[0]-q1[0])/2)
                midY = q1[1] + int((q2[1]-q1[1])/2)
                if(midX>0 and midX<maxX and midY>0 and midY<maxY and self.map_array[midX][midY] == 1):
                    self.samples.append((midX,midY))
        #self.samples.append((0, 0))


    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots()
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw graph
        # get node position (swap coordinates)
        node_pos = np.array(self.samples)[:, [1, 0]]
        pos = dict( zip( range( len(self.samples) ), node_pos) )
        pos['start'] = (self.samples[-2][1], self.samples[-2][0])
        pos['goal'] = (self.samples[-1][1], self.samples[-1][0])
        
        # draw constructed graph
        nx.draw(self.graph, pos, node_size=3, node_color='y', edge_color='y' ,ax=ax)

        # If found a path
        if self.path:
            # add temporary start and goal edge to the path
            final_path_edge = list(zip(self.path[:-1], self.path[1:]))
            nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=self.path, node_size=8, node_color='b')
            nx.draw_networkx_edges(self.graph, pos=pos, edgelist=final_path_edge, width=2, edge_color='b')

        # draw start and goal
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['start'], node_size=12,  node_color='g')
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['goal'], node_size=12,  node_color='r')

        # show image
        plt.axis('on')
        ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
        plt.show()


    def sample(self, n_pts=1000, sampling_method="uniform"):
        '''Construct a graph for PRM
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            sampling_method - name of the chosen sampling method

        Sample points, connect, and add nodes and edges to self.graph
        '''
        # Initialize before sampling
        self.samples = []
        self.graph.clear()
        self.path = []

        # Sample methods
        if sampling_method == "uniform":
            self.uniform_sample(n_pts)
        elif sampling_method == "random":
            self.random_sample(n_pts)
        elif sampling_method == "gaussian":
            self.gaussian_sample(n_pts)
        elif sampling_method == "bridge":
            self.bridge_sample(n_pts)

        ### YOUR CODE HERE ###

        # Find the pairs of points that need to be connected
        # and compute their distance/weight.
        # Store them as
        # pairs = [(p_id0, p_id1, weight_01), (p_id0, p_id2, weight_02), 
        #          (p_id1, p_id2, weight_12) ...]
        pairs = []
        kdTree = spatial.KDTree(self.samples)
        for i in range (len(self.samples)):
            dis, id = kdTree.query(self.samples[i],k=10)
            for j in range(10):
                if(id[j] == i):
                    continue
                if(self.check_collision(self.samples[i],self.samples[id[j]])):
                    continue
                dis = self.dis(self.samples[i],self.samples[id[j]])
                pairs.append((i,id[j],dis))

        # Use sampled points and pairs of points to build a graph.
        # To add nodes to the graph, use
        # self.graph.add_nodes_from([p_id0, p_id1, p_id2 ...])
        # To add weighted edges to the graph, use
        # self.graph.add_weighted_edges_from([(p_id0, p_id1, weight_01), 
        #                                     (p_id0, p_id2, weight_02), 
        #                                     (p_id1, p_id2, weight_12) ...])
        # 'p_id' here is an integer, representing the order of 
        # current point in self.samples
        # For example, for self.samples = [(1, 2), (3, 4), (5, 6)],
        # p_id for (1, 2) is 0 and p_id for (3, 4) is 1.
        self.graph.add_nodes_from([i for i in range(len(self.samples))])
        self.graph.add_weighted_edges_from(pairs)

        # Print constructed graph information
        n_nodes = self.graph.number_of_nodes()
        n_edges = self.graph.number_of_edges()
        print("The constructed graph has %d nodes and %d edges" %(n_nodes, n_edges))


    def search(self, start, goal):
        '''Search for a path in graph given start and goal location
        arguments:
            start - start point coordinate [row, col]
            goal - goal point coordinate [row, col]

        Temporary add start and goal node, edges of them and their nearest neighbors
        to graph for self.graph to search for a path.
        '''
        # Clear previous path
        self.path = []

        # Temporarily add start and goal to the graph
        self.samples.append(start)
        self.samples.append(goal)
        # start and goal id will be 'start' and 'goal' instead of some integer
        self.graph.add_nodes_from(['start', 'goal'])
        ### YOUR CODE HERE ###
        # Find the pairs of points that need to be connected
        # and compute their distance/weight.
        # You could store them as
        # start_pairs = [(start_id, p_id0, weight_s0), (start_id, p_id1, weight_s1), 
        #                (start_id, p_id2, weight_s2) ...]
        start_pairs = []
        goal_pairs = []
        kdTree = spatial.KDTree(self.samples)
        dis, id = kdTree.query(start,k=10)
        for j in range(10):
                if(id[j] == 'start'):
                    continue
                if(self.check_collision(start,self.samples[id[j]])):
                    continue
                dis = self.dis(start,self.samples[id[j]])
                start_pairs.append(('start',id[j],dis))
        dis, id = kdTree.query(goal,k=10)
        for j in range(10):
                if(id[j] == 'goal'):
                    continue
                if(self.check_collision(goal,self.samples[id[j]])):
                    continue
                dis = self.dis(goal,self.samples[id[j]])
                start_pairs.append(('goal',id[j],dis))

        # Add the edge to graph
        self.graph.add_weighted_edges_from(start_pairs)
        self.graph.add_weighted_edges_from(goal_pairs)
        
        # Seach using Dijkstra
        try:
            self.path = nx.algorithms.shortest_paths.weighted.dijkstra_path(self.graph, 'start', 'goal')
            path_length = nx.algorithms.shortest_paths.weighted.dijkstra_path_length(self.graph, 'start', 'goal')
            print("The path length is %.2f" %path_length)
        except nx.exception.NetworkXNoPath:
            print("No path found")
        
        # Draw result
        self.draw_map()

        # Remove start and goal node and their edges
        self.samples.pop(-1)
        self.samples.pop(-1)
        self.graph.remove_nodes_from(['start', 'goal'])
        self.graph.remove_edges_from(start_pairs)
        self.graph.remove_edges_from(goal_pairs)
        