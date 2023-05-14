# Basic searching algorithms

# Class for each node in the grid
class Node:
    def __init__(self, row, col, is_obs, h):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.is_obs = is_obs  # obstacle?
        self.g = None         # cost to come (previous g + moving cost)
        self.h = h            # heuristic
        self.cost = None      # total cost (depend on the algorithm)
        self.parent = None    # previous node

def dijkstra(grid, start, goal):
    '''Return a path found by Dijkstra alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dij_path, dij_steps = dijkstra(grid, start, goal)
    It takes 10 steps to find a path using Dijkstra
    >>> dij_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False
    rows,cols = len(grid),len(grid[0])
    nodeGrid = []
    for i in range(rows):
        temp = []
        for j in range(cols):
            temp.append(None)
        nodeGrid.append(temp)
    Q = {}
    for i in range(rows):
        for j in range(cols):
            if(grid[i][j] == 0):
                nodeGrid[i][j] = Node(i,j,False,0)
                nodeGrid[i][j].cost = 1000000
            else:
                nodeGrid[i][j] = Node(i,j,True,0)
                nodeGrid[i][j].cost = 1000000
            Q[(i,j)] = nodeGrid[i][j].cost

    Q[(start[0],start[1])] = 0
    while(len(Q) != 0):
        u = next(iter(Q))
        x,y = u
        for key,value in Q.items():
            if(value < Q[u]):
                u = key
        steps += 1
        if(u[0]==goal[0] and u[1]==goal[1]):
            found = True
            break
        dist = Q.pop(u)
        right = (u[0],u[1]+1)
        down = (u[0]+1,u[1])
        left = (u[0],u[1]-1)
        up = (u[0]-1,u[1])
        if(right[1]>=0 and right[1]<cols and not nodeGrid[right[0]][right[1]].is_obs):
            if right in Q.keys():    
                alt = dist + 1
                if(alt < Q[right]):
                    Q[right] = alt
                    nodeGrid[right[0]][right[1]].cost = alt
                    nodeGrid[right[0]][right[1]].parent = nodeGrid[u[0]][u[1]]
        if(down[0]>=0 and down[0]<rows and not nodeGrid[down[0]][down[1]].is_obs):
            if down in Q.keys():
                alt = dist + 1
                if(alt < Q[down]):
                    Q[down] = alt
                    nodeGrid[down[0]][down[1]].cost = alt
                    nodeGrid[down[0]][down[1]].parent = nodeGrid[u[0]][u[1]]
        if(left[1]>=0 and left[1]<cols and not nodeGrid[left[0]][left[1]].is_obs):
            if left in Q.keys():
                alt = dist + 1
                if(alt < Q[left]):
                    Q[left] = alt
                    nodeGrid[left[0]][left[1]].cost = alt
                    nodeGrid[left[0]][left[1]].parent = nodeGrid[u[0]][u[1]]
        if(up[0]>=0 and up[0]<rows and not nodeGrid[up[0]][up[1]].is_obs):
            if up in Q.keys():
                alt = dist + 1
                if(alt < Q[up]):
                    Q[up] = alt
                    nodeGrid[up[0]][up[1]].cost = alt
                    nodeGrid[up[0]][up[1]].parent = nodeGrid[u[0]][u[1]]


    if found:
        print(f"It takes {steps} steps to find a path using Dijkstra")
        while( not (u[0]==start[0] and u[1]==start[1])):
            path.append([u[0],u[1]])
            node = nodeGrid[u[0]][u[1]].parent
            u = (node.row,node.col)
        path.append([u[0],u[1]])
        path.reverse()
    else:
        print("No path found")
    return path, steps


def astar(grid, start, goal):
    '''Return a path found by A* alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> astar_path, astar_steps = astar(grid, start, goal)
    It takes 7 steps to find a path using A*
    >>> astar_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False
    rows,cols = len(grid),len(grid[0])
    nodeGrid = []
    for i in range(rows):
        temp = []
        for j in range(cols):
            temp.append(None)
        nodeGrid.append(temp)
    for i in range(rows):
        for j in range(cols):
            if(grid[i][j] == 0):
                nodeGrid[i][j] = Node(i,j,False,ManhattanDistance(i,j,goal[0],goal[1]))
                nodeGrid[i][j].cost = 1000000
            else:
                nodeGrid[i][j] = Node(i,j,True,ManhattanDistance(i,j,goal[0],goal[1]))
                nodeGrid[i][j].cost = 1000000

    nodeGrid[start[0]][start[1]].g = 0
    openlist = []
    closedList = []
    openlist.append([start[0],start[1]])
    while len(openlist) != 0:
        u = openlist[0]
        for node in openlist:
            if(nodeGrid[node[0]][node[1]].cost < nodeGrid[u[0]][u[1]].cost):
                u = node
        u = openlist.pop(openlist.index(u))
        closedList.append(u)
        steps += 1
        if(u[0]==goal[0] and u[1]==goal[1]):
            found = True
            break
        q = nodeGrid[u[0]][u[1]]
        right = [u[0],u[1]+1]
        down = [u[0]+1,u[1]]
        left = [u[0],u[1]-1]
        up = [u[0]-1,u[1]]

        if(right[0]>=0 and right[0]<rows and right[1]>=0 and right[1]<cols and not nodeGrid[right[0]][right[1]].is_obs):
            if right not in closedList:
                rightNode = nodeGrid[right[0]][right[1]]
                cost = q.g + 1 + rightNode.h
                if cost < rightNode.cost:
                    rightNode.cost = cost
                    rightNode.parent = u
                    rightNode.g = q.g + 1
                    nodeGrid[right[0]][right[1]] = rightNode
                    openlist.append([right[0],right[1]])

        if(down[0]>=0 and down[0]<rows and down[1]>=0 and down[1]<cols and not nodeGrid[down[0]][down[1]].is_obs):
            if down not in closedList:
                downNode = nodeGrid[down[0]][down[1]]
                cost = q.g + 1 + downNode.h
                if cost < downNode.cost:
                    downNode.cost = cost
                    downNode.parent = u
                    downNode.g = q.g + 1
                    nodeGrid[down[0]][down[1]] = downNode
                    openlist.append([down[0],down[1]])

        if(left[0]>=0 and left[0]<rows and left[1]>=0 and left[1]<cols and not nodeGrid[left[0]][left[1]].is_obs):
            if left not in closedList:
                leftNode = nodeGrid[left[0]][left[1]]
                cost = q.g + 1 + leftNode.h
                if cost < leftNode.cost:
                    leftNode.cost = cost
                    leftNode.parent = u
                    leftNode.g = q.g + 1
                    nodeGrid[left[0]][left[1]] = leftNode
                    openlist.append([left[0],left[1]])

        if(up[0]>=0 and up[0]<rows and up[1]>=0 and up[1]<cols and not nodeGrid[up[0]][up[1]].is_obs):
            if up not in closedList:
                upNode = nodeGrid[up[0]][up[1]]
                cost = q.g + 1 + upNode.h
                if cost < upNode.cost:
                    upNode.cost = cost
                    upNode.parent = u
                    upNode.g = q.g + 1
                    nodeGrid[up[0]][up[1]] = upNode
                    openlist.append([up[0],up[1]])

    if found:
        print(f"It takes {steps} steps to find a path using A*")
        while( not (u[0]==start[0] and u[1]==start[1])):
            path.append([u[0],u[1]])
            node = nodeGrid[u[0]][u[1]].parent
            u = (node[0],node[1])
        path.append([u[0],u[1]])
        path.reverse()
    else:
        print("No path found")
    return path, steps


def ManhattanDistance(x1,y1,x2,y2):
    dist = abs(x1-x2) + abs(y1-y2)
    return dist


# Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples
    # Test all the functions
    testmod()
