import matplotlib.pyplot as plt
import numpy as np

# Create a Dictionary of maps, in order for the user to get easy access to each map via integers (Global variable - for access)
map_options = {
    1:  {
        'name' : 'Hanoi',
        'grid' : [
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]
    },

    2:  {
        'name' : 'Sai Gon',
        'grid' : [
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 1, 1, 1, 1, 0],
        [0, 1, 0, 0, 0, 1, 0, 0, 0, 0],
        [0, 1, 1, 1, 1, 1, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 0, 0, 0]]
    },

    3:  {
        'name' : 'Da Lat',
        'grid' : [
        [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 1, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0]]
    }
} 

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None): # A constructor that gets the 3 variables of a node, (self - the node itself), (parent - the previous node that it derives from), (position - the coordinates of the node)
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
        open_list_states.pop(current_index) # Appending the explored nodes    
        closed_list.append(current_node)
        closed_list_states.append(current_node.position) # Append the explored nodes/nodes that are not chosen

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
            if child.position in open_list_states:
                continue 

            # Create the f, g, and h values
            child.g = current_node.g + STEP_COST
            child.h = max(abs(child.position[0] - end_node.position[0]), abs(child.position[1] - end_node.position[1])) # The Chebyshev Distance caclculation method to the heuristic value
            child.f = child.g + child.h

            # Update f(n)
            for index, item in enumerate(open_list):
                if child.position == item.position and child.f < item.f:
                    open_list.pop(index) # Remove the node to replace with child later   

            # Add the child to the open list
            open_list.append(child)
            open_list_states.append(child.position); 

def display_maps(id, replacements):
    # Retrieve the map data from the dictionary
    if id in map_options:
        map_grid = map_options[id]['grid']
        map_name = map_options[id]['name']

        # Print the map name
        print(f"{map_name}:")

        # Iterate over each row in the grid and print it
        for row in map_grid:
            print(' '.join([replacements.get(value, str(value)) for value in row]))
            print()
    else:
        print(f"Map ID {id} not found.")

def path_visualize(path, selected_map, start_position, end_position):

    # Saving the grid and name of a map to separate variables
    map_grid = np.array(selected_map['grid'])
    map_name = selected_map['name']

    # Create a plot figure for the map:
    plt.figure(figsize=(10,10))
    plt.imshow(map_grid, cmap = 'Grays', origin = 'upper') # Create and format a map figure
    plt.title(f"Path Visualization on the {map_name} Grid", fontsize=16) # Map title
    plt.grid(False) # Turn off the grid and axis
    plt.axis('on')

    # Show the path on the map
    x_path = [p[1] for p in path] # Plot the x coordinates
    y_path = [p[0] for p in path] # Plot the y coordinates
    plt.plot(x_path, y_path)

    # Highlight the start and end positions
    plt.scatter(start_position[0], start_position[1], color='green', s=200, label='Start')  # Start position (green)
    plt.scatter(end_position[1], end_position[0], color='red', s=200, label='End')          # End position (red)

    # Show the graph
    plt.show()

def main():
    # A welcoming message for the user and print out the mazes name and their grid for user to see
    welcome_message = """
    ****WELCOME THE THE A* MAZE ALGORITHM TEST!!!****
    This A* algorithm program searches for the best path in a map!
    Keys:
    '.' = Passable spots
    '|' = Obstacle - You can imagine a wall, unpassable building, etc.
    """

    print(welcome_message)                    
    print("Here are some of the maps that you can try")

    # replacements instead of the number 0 and 1, I used symbols instead
    replacements = {0: '.', 1: '|'}    

    # Display the map options:
    print("Choose your map in the following options: \n")
    for id in range(1, 4):
        display_maps(id, replacements)

    maze_id = int(input("Select the maze you want to try (from 1 - 3): ")) # Get input from the user's desired map selection
    while maze_id not in map_options: # Error handling any miss input
        print("Invalid map option, please select again!")
        maze_id = int(input("Select the maze you want to try: "))

    # Saving map option:
    selected_map = map_options[maze_id]   
    map_name = map_options[maze_id]['name']
    map_grid = map_options[maze_id]['grid']

    # Using the split function, enter the start and end point in x y coordinates:
    start_input = input("Select your start point (x, y): ")
    start_points = start_input.split()
    start_x = int(start_points[0])
    start_y = int(start_points[1])

    # Ask the user input 
    end_input = input("Select your end point (x, y): ")
    end_points = end_input.split()
    end_x = int(end_points[0])
    end_y = int(end_points[1])

    # Saving the two points in a tuple for any path related functions
    start_position = (start_x, start_y)
    end_position = (end_x, end_y)

    path = astar(map_grid, start_position, end_position) # Start the function by passing the map_grid, and start to end point

    # Find the Evaluation report:
    print("\n****EVALUATION REPORT****")
    print(f"- Map:                {map_name}, ID: {maze_id}")
    print(f"- Path cost:          {len(path)-1}")
    print(f"- Starting position:  {start_position}")
    print(f"- End position:       {end_position}")
    print(f"- Path:               {path}") # Print the path

    # Path visualization:
    path_visualize(path, selected_map, start_position, end_position)

if __name__ == '__main__':
    main()