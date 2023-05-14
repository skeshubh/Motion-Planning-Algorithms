# Basic searching algorithms

# Class for each node in the grid
class Node:
    def __init__(self, row, col, is_obs, h):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.is_obs = is_obs  # obstacle?
        self.cost = None      # total cost (depend on the algorithm)
        self.parent = None    # previous node


def bfs(grid, start, goal):
    '''Return a path found by BFS alogirhm 
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
    >>> bfs_path, bfs_steps = bfs(grid, start, goal)
    It takes 10 steps to find a path using BFS
    >>> bfs_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False
    rows,cols = len(grid),len(grid[0])    #total rows and columns in grid
    visited = []    #list to keep track of already visited nodes
    queue = []    #queue to keep track of next nodes to explore
    u = Node(start[0],start[1],False,None)    #start node
    queue.append(u)    #appending the start node in queue to start exploring
    while(len(queue) != 0):    #exploring till there's no nodes left
        u = queue.pop(0)    #taking out the first element from queue (popping)
        steps += 1    #incrementing the step count
        if(u.row==goal[0] and u.col==goal[1]):    #if current node is goal then break
            found = True
            break
        visited.append(u)    #mark current node as visited
        if(canVisit(u.row,u.col+1,rows,cols,visited)):    #checking if you can visit the right[0,+1] node
            right = generateNode(u.row,u.col+1,grid)    #generating the node
            if(not right.is_obs):    #if right node is not an obstacle put it in queue to explore
                visited.append(right)
                right.parent = u
                queue.append(right)
        if(canVisit(u.row+1,u.col,rows,cols,visited)):    #checking if you can visit the down[+1,0] node
            down = generateNode(u.row+1,u.col,grid)    #generating node
            if(not down.is_obs):    #if down node is not an onstacle put it in queue to explore
                visited.append(down)
                down.parent = u
                queue.append(down)
        if(canVisit(u.row,u.col-1,rows,cols,visited)):    #checking if you can visit the left[0,-1] node
            left = generateNode(u.row,u.col-1,grid)    #generating node
            if(not left.is_obs):    #if left node is not an onstacle put it in queue to explore
                visited.append(left)
                left.parent = u
                queue.append(left)
        if(canVisit(u.row-1,u.col,rows,cols,visited)):    #checking if you can visit the up[-1,0] node
            up = generateNode(u.row-1,u.col,grid)    #generating node
            if(not up.is_obs):    #if left node is not an onstacle put it in queue to explore
                visited.append(up)
                up.parent = u
                queue.append(up)
    if found:    #if solution is found
        print(f"It takes {steps} steps to find a path using BFS")
        while( not (u.row==start[0] and u.col==start[1])):    #exploring the path in reverse starting from the goal node
            path.append([u.row,u.col])
            u = u.parent
        path.append([u.row,u.col])
        path.reverse()
    else:
        print("No path found")
    return path, steps

def generateNode(row,col,grid):    #method to generate a Node object based on if the node is an obstacle or not
    if(grid[row][col] == 1):
        return Node(row,col,True,None)
    else:
        return Node(row,col,False,None)

def canVisit(row,col,rows,cols,visited):    #method to check if the node is within bounds of the defined grid and is not visited yet
    if(row>=0 and row<rows and col>=0 and col<cols):
        check = True
        for n in visited:
            if(row==n.row and col==n.col):
                check = False
                break
        return check
    else:
        return False

def dfs(grid, start, goal):
    '''Return a path found by DFS alogirhm 
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
    >>> dfs_path, dfs_steps = dfs(grid, start, goal)
    It takes 9 steps to find a path using DFS
    >>> dfs_path
    [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2], [2, 3], [3, 3], [3, 2], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False
    rows,cols = len(grid),len(grid[0])    #total rows and columns in grid
    visited = []    #list to keep track of already visited nodes
    u = Node(start[0],start[1],False,None)    #start node
    visited.append(u)    #start node is marked explored
    u,found,steps = DfsVisit(grid,steps,u,goal,visited)    #start dfs algorithm
    if found:    #is solution is found
        print(f"It takes {steps} steps to find a path using DFS")
        while( not (u.row==start[0] and u.col==start[1])):    #exploring the path in reverse starting from the goal node
            path.append([u.row,u.col])
            u = u.parent
        path.append([u.row,u.col])
        path.reverse()
    else:
        print("No path found")
    return path, steps

def DfsVisit(grid,steps,node,goal,visited):    #recursive call algorithm of DFS
    steps += 1    #incrementing the step count
    if(node.row==goal[0] and node.col==goal[1]):    #if current node is goal then break
        return node,True,steps
    if(canVisit(node.row,node.col+1,len(grid),len(grid[0]),visited)):    #checking if you can visit the right[0,+1] node
        right = generateNode(node.row,node.col+1,grid)    #generating the node
        if not right.is_obs:    #if right node is not an obstacle explore it 
            visited.append(right)
            right.parent = node
            right,found,steps = DfsVisit(grid,steps,right,goal,visited)
            if found:    #if right node is the solution then return
                return right,found,steps
    if(canVisit(node.row+1,node.col,len(grid),len(grid[0]),visited)):    #checking if you can visit the down[+1,0] node
        down = generateNode(node.row+1,node.col,grid)    #generating the node
        if not down.is_obs:    #if down node is not an obstacle explore it 
            visited.append(down)
            down.parent = node
            down,found,steps = DfsVisit(grid,steps,down,goal,visited)
            if found:    #if down node is the solution then return
                return down,found,steps
    if(canVisit(node.row,node.col-1,len(grid),len(grid[0]),visited)):    #checking if you can visit the left[0,-1] node
        left = generateNode(node.row,node.col-1,grid)    #generating the node
        if not left.is_obs:    #if left node is not an obstacle explore it 
            visited.append(left)
            left.parent = node
            left,found,steps = DfsVisit(grid,steps,left,goal,visited)
            if found:    #if left node is the solution then return
                return left,found,steps
    if(canVisit(node.row-1,node.col,len(grid),len(grid[0]),visited)):    #checking if you can visit the up[-1,0] node
        up = generateNode(node.row-1,node.col,grid)    #generating the node
        if not up.is_obs:    #if up node is not an obstacle explore it 
            visited.append(up)
            up.parent = node
            up,found,steps = DfsVisit(grid,steps,up,goal,visited)
            if found:    #if up node is the solution then return
                return up,found,steps
    return node,False,steps    #if no nodes can be visited then return to calling method

# Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples
    # Test all the functions
    testmod()
