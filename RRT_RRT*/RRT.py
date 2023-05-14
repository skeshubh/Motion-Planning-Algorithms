# Standard Algorithm Implementation
# Sampling-based Algorithms RRT and RRT*

import matplotlib.pyplot as plt
import numpy as np


# Class for each tree node
class Node:
    def __init__(self, row, col):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.parent = None    # parent node
        self.cost = 0.0       # cost


# Class for RRT
class RRT:
    # Constructor
    def __init__(self, map_array, start, goal):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.start = Node(start[0], start[1]) # start node
        self.goal = Node(goal[0], goal[1])    # goal node
        self.vertices = []                    # list of nodes
        self.found = False                    # found flag
        self.step_size = 15
        

    def init_map(self):
        '''Intialize the map before each search
        '''
        self.found = False
        self.vertices = []
        self.vertices.append(self.start)

    
    def dis(self, node1, node2):
        '''Calculate the euclidean distance between two nodes
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            euclidean distance between two nodes
        '''
        ### YOUR CODE HERE ###
        distance = np.sqrt(np.square(node1.row - node2.row) + np.square(node1.col - node2.col))
        return distance

    
    def check_collision(self, node1, node2):
        '''Check if the path between two nodes collide with obstacles
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            True if the new node is valid to be connected
        '''
        ### YOUR CODE HERE ###
        x = node1.row
        y = node1.col
        dx = abs(node2.row - node1.row)
        dy = abs(node2.col - node1.col)
        s1 = 0
        s2 = 0
        if(node2.row - node1.row > 0):
            s1 = 1
        elif(node2.row - node1.row < 0):
            s1 = -1
        if(node2.col - node1.col > 0):
            s2 = 1
        elif(node2.col - node1.col < 0):
            s2 = -1
        interchange = 0
        if(dy > dx):
            temp = dx
            dx = dy
            dy = temp
            interchange = 1
        E = 2*dy - dx
        A = 2*dy
        B = 2*dy - 2*dx
        points = []
        points.append([x,y])
        for i in range(1,dx+1):
            if(E < 0):
                if(interchange == 1):
                    y = y + s2
                else:
                    x = x + s1
                E = E + A
            else:
                y = y + s2
                x = x + s1
                E = E + B
            points.append([x,y])
        isSafe = True
        for point in points:
            if(point[0]>=0 and point[0]<len(self.map_array) and point[1]>=0 and point[1]<len(self.map_array[0]) and self.map_array[point[0]][point[1]] == 0):
                isSafe = False
                break
        return isSafe


    def get_new_point(self, goal_bias):
        '''Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point

        return:
            point - the new point
        '''
        ### YOUR CODE HERE ###
        prob = np.random.randint(1,1000)
        if goal_bias*1000 >= prob:
            return Node(self.goal.row,self.goal.col)
        else:
            randCol = 0
            randRow = 0
            while(True):
                randRow = np.random.randint(0,self.size_row)
                randCol = np.random.randint(0,self.size_col)
                checkVertices = False
                for node in self.vertices:
                    if node.col == randCol and node.row == randRow:
                        checkVertices = True
                        break
                if checkVertices:
                    continue
                else:
                    break
            return Node(randRow,randCol)

    
    def get_nearest_node(self, point):
        '''Find the nearest node in self.vertices with respect to the new point
        arguments:
            point - the new point

        return:
            the nearest node
        '''
        ### YOUR CODE HERE ###
        minNode = self.vertices[0]
        minDist = self.dis(point, self.vertices[0])
        for node in self.vertices:
            dist = self.dis(point,node)
            if(dist < minDist):
                minNode = node
                minDist = dist
        return minNode


    def get_neighbors(self, new_node, neighbor_size):
        '''Get the neighbors that are within the neighbor distance from the node
        arguments:
            new_node - a new node
            neighbor_size - the neighbor distance

        return:
            neighbors - a list of neighbors that are within the neighbor distance 
        '''
        ### YOUR CODE HERE ###
        neighbors = []
        for node in self.vertices:
            dist = self.dis(new_node,node)
            if(dist < neighbor_size):
                neighbors.append(node)
        return neighbors


    def rewire(self, new_node, neighbors):
        '''Rewire the new node and all its neighbors
        arguments:
            new_node - the new node
            neighbors - a list of neighbors that are within the neighbor distance from the node

        Rewire the new node if connecting to a new neighbor node will give least cost.
        Rewire all the other neighbor nodes.
        '''
        ### YOUR CODE HERE ###
        for node in neighbors:
            newCost = node.cost + self.dis(node,new_node)
            if (newCost < new_node.cost) and self.check_collision(node,new_node):
                for i in range(0,len(self.vertices)):
                    if self.vertices[i].row == new_node.row and self.vertices[i].col == new_node.col:
                        self.vertices[i].cost = newCost
                        self.vertices[i].parent = node

        for node in neighbors:
            newCost = new_node.cost + self.dis(new_node,node)
            if (newCost < node.cost) and self.check_collision(node,new_node):
                for j in range(0,len(self.vertices)):
                    if self.vertices[j].row == node.row and self.vertices[j].col == node.col:
                        self.vertices[j].cost = newCost
                        self.vertices[j].parent = new_node

    
    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots(1)
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw Trees or Sample points
        for node in self.vertices[1:-1]:
            plt.plot(node.col, node.row, markersize=3, marker='o', color='y')
            plt.plot([node.col, node.parent.col], [node.row, node.parent.row], color='y')
        
        # Draw Final Path if found
        if self.found:
            cur = self.goal
            while cur.col != self.start.col or cur.row != self.start.row:
                plt.plot([cur.col, cur.parent.col], [cur.row, cur.parent.row], color='b')
                cur = cur.parent
                plt.plot(cur.col, cur.row, markersize=3, marker='o', color='b')

        # Draw start and goal
        plt.plot(self.start.col, self.start.row, markersize=5, marker='o', color='g')
        plt.plot(self.goal.col, self.goal.row, markersize=5, marker='o', color='r')

        # show image
        plt.show()


    def RRT(self, n_pts=1000):
        '''RRT main search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        In each step, extend a new node if possible, and check if reached the goal
        '''
        # Remove previous result
        self.init_map()

        ### YOUR CODE HERE ###

        # In each step,
        # get a new point, 
        # get its nearest node, 
        # extend the node and check collision to decide whether to add or drop,
        # if added, check if reach the neighbor region of the goal.
        goal_bias = 0.15
        for i in range(0,n_pts):
            newPoint = self.get_new_point(goal_bias)
            if(self.map_array[newPoint.row][newPoint.col] == 0):
                continue
            nearestNode = self.get_nearest_node(newPoint)
            if(self.check_collision(newPoint,nearestNode)):
                dist = self.dis(nearestNode,newPoint)
                if dist > self.step_size:
                    t = self.step_size/dist
                    newPoint.row = round((1-t)*nearestNode.row + t*newPoint.row)
                    newPoint.col = round((1-t)*nearestNode.col + t*newPoint.col)
                if not self.check_collision(nearestNode,newPoint) or self.map_array[newPoint.row][newPoint.col]==0:
                    continue
                dist = self.dis(nearestNode,newPoint)
                newPoint.cost = nearestNode.cost + dist
                newPoint.parent = nearestNode
                self.vertices.append(newPoint)
                if(newPoint.row==self.goal.row and newPoint.col==self.goal.col):
                    self.found = True
                    self.goal.parent = nearestNode
                    self.goal.cost = newPoint.cost
                    break
        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")
        
        # Draw result
        self.draw_map()


    def RRT_star(self, n_pts=1000, neighbor_size=80):
        '''RRT* search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            neighbor_size - the neighbor distance
        
        In each step, extend a new node if possible, and rewire the node and its neighbors
        '''
        # Remove previous result
        self.init_map()

        ### YOUR CODE HERE ###

        # In each step,
        # get a new point, 
        # get its nearest node, 
        # extend the node and check collision to decide whether to add or drop,
        # if added, rewire the node and its neighbors,
        # and check if reach the neighbor region of the goal if the path is not found.
        #print("goal = ",self.goal.row,self.goal.col)
        realGoalRow = self.goal.row
        realGoalCol = self.goal.col
        goal_bias = 0.15
        for i in range(0,n_pts):
            newPoint = self.get_new_point(goal_bias)
            if(self.map_array[newPoint.row][newPoint.col] == 0):
                continue
            nearestNode = self.get_nearest_node(newPoint)
            if(self.check_collision(newPoint,nearestNode)):
                dist = self.dis(nearestNode,newPoint)
                if dist > self.step_size:
                    t = self.step_size/dist
                    row = round((1-t)*nearestNode.row + t*newPoint.row)
                    col = round((1-t)*nearestNode.col + t*newPoint.col)
                    newPoint.row = row
                    newPoint.col = col
                if not self.check_collision(nearestNode,newPoint) or self.map_array[newPoint.row][newPoint.col]==0:
                    continue
                self.goal.row = realGoalRow
                self.goal.col = realGoalCol
                dist = self.dis(nearestNode,newPoint)
                newPoint.cost = nearestNode.cost + dist
                newPoint.parent = nearestNode
                self.vertices.append(newPoint)
                neighbors = self.get_neighbors(newPoint,neighbor_size)
                self.rewire(newPoint,neighbors)
                if(newPoint.row==self.goal.row and newPoint.col==self.goal.col):
                    self.found = True
                    self.goal.parent = nearestNode
                    self.goal.cost = newPoint.cost
                    break

        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")

        # Draw result
        self.draw_map()
