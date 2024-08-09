# Debugged version of Astar_maze.py

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0 # PATH-COST to the node
        self.h = 0 # heuristic to the goal: straight-line distance hueristic
        self.f = 0 # evaluation function f(n) = g(n) + h(n)

    def __eq__(self, other):
        return self.position == other.position


def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    STEP_COST = 1 # qAdd: For generalization

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []    # frontier
    closed_list = []  # explored
    open_list_states = []    # qAdd
    closed_list_states = []  # qAdd

    # Add the start node
    open_list.append(start_node)
    open_list_states.append(start_node.position)

    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node (qCom: Node with the least f)
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            #if item.g < current_node.g: # qNote: Uniform-cost search
            #if item.h < current_node.h: # qNote: Best-first search
            if item.f < current_node.f: # qNote: a A-star search
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        open_list_states.pop(current_index)    
        closed_list.append(current_node)
        closed_list_states.append(current_node.position)

        # Check if found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Expansion: Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:
            if child.position in closed_list_states:
                continue

            # Create the f, g, and h values
            child.g = current_node.g + STEP_COST
            child.h = (((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2))**0.5 # qEdit
            child.f = child.g + child.h

            # Update f(n)
            for index, item in enumerate(open_list):
                if child.position == item.position and child.f < item.f:
                    open_list.pop(index) # Remove the node to replace with child later   
                    
                    # Add the child to the open list (update new child with new f(n))
                    open_list.append(child)
                    open_list_states.append(child.position);  

            if child.position not in open_list_states:                                       
                # Add the child to the open list
                open_list.append(child)
                open_list_states.append(child.position); 
         

def main():
    maze_id = 2
    if maze_id==1:
        maze = [[0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]
        start = (0, 0)
        end = (7, 6)

    elif maze_id==2:
        maze = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 1, 1, 1, 1, 0],
                [0, 1, 0, 0, 0, 1, 0, 0, 0, 0],
                [0, 1, 1, 1, 1, 1, 0, 0, 0, 0],
                [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 1, 0, 0, 1, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 1, 0, 0, 0]]
        start = (0, 0)
        end = (8, 9)
    
    path = astar(maze, start, end)

    if path is not None:
        print('\nPath:',path)
        print('Path length:',len(path)-1)  # Optimal path length (found by UCS): 14 actions
        print('\n')
    else:
        print('\nNo solution found!\n')   

if __name__ == '__main__':
    main()
