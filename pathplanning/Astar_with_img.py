# https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2


import numpy as np
import cv2
import time
import img2binList as i2b
import random
import operator

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0
        # new distance cost
        self.dc = 0

    def __eq__(self, other):
        return self.position == other.position


def discost(maze, x, y):
    """Calculate the distance cost according to neighbor obstacles"""
    """If the distance(in grid scale) between current node and obstacle is less than 4, 
    the distance cost will be given according to that distance"""
    distance_cost = 0
    eight_neighbors = [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    for new_position in eight_neighbors:
        node_position = (x + new_position[0], y + new_position[1])
        if maze[node_position[0]][node_position[1]] == 1:
            distance_cost += 4
            break
        else:
            for new_position2 in eight_neighbors:
                node_position2 = (node_position[0] + new_position2[0], node_position[1] + new_position2[1])
                if maze[node_position2[0]][node_position2[1]] == 1:
                    distance_cost += 3
                    break
                else:
                    for new_position3 in eight_neighbors:
                        node_position3 = (node_position2[0] + new_position3[0], node_position2[1] + new_position3[1])
                        if maze[node_position3[0]][node_position3[1]] == 1:
                            distance_cost += 2
                            break
                        else:
                            for new_position4 in eight_neighbors:
                                node_position4 = (node_position3[0] + new_position4[0], node_position3[1] + new_position4[1])
                                if maze[node_position4[0]][node_position4[1]] == 1:
                                    distance_cost += 1
    return distance_cost


def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""
    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []
    # Check if start or end node is on the obstacle
    if maze[start[0]][start[1]] == 1 or maze[end[0]][end[1]] == 1:
        print("Start or End node is not walkable terrain")
    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:
        # Get the current node
        # Refresh the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal node
        if current_node == end_node:
            path = []
            current = current_node
            # accumulate parents nodes to draw the path
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

            # Get node position (8 neighborhoods)
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Avoid infinite loop by checking closed list
            if Node(current_node, node_position) in closed_list:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:
            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    break
            else:
                # Create the f, g, and h values
                # child.g = (current_node.g + ((child.position[0]-current_node.position[0])**2+(child.position[1]-current_node.position[1])**2))
                child.g = current_node.g + 1
                child.h = (((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2))
                # New cost 'distance cost' as dc
                # The weight of the distance cost has been set to make the path at least 3 grid away from the obstacles.
                child.dc = 1000 * discost(maze, child.position[0], child.position[1])
                child.f = child.g + child.h + child.dc

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g >= open_node.g:
                    break
            else:
                # Add the child to the open list
                open_list.append(child)


def convert2meter(path, scale=0.5):
    """convert the path in meter scale"""
    """in general, one grid represent 0.5 meter"""
    path_list = [list(elem) for elem in path]
    metered_path = []
    for grid in path_list:
        metered_grid = [i * scale for i in grid]
        metered_path.append(metered_grid)
    return metered_path



def main():
    # Running Time Check
    starttime = time.time()

    # Convert map image to binary list
    maze = i2b.img2binList(lenWidth=500.0, GRID_SIZE=5, verbose=0)

    # Start and End point setting
    start = (random.randrange(80, 90), random.randrange(10, 20))
    end = (random.randrange(10, 20), random.randrange(80, 90))
    print("Start =", start, '\n', "End =", end)

    # Procedure Checking
    print(" ", "Path planning Proceeding...", " ", sep='\n')
    path = astar(maze, start, end)
    print("Path planning Succeed")
    print("time :", time.time() - starttime)
    print("Path : ", path)
    print("Meter scale Path : ", convert2meter(path))

    # Visualizing binary map and generated path
    showmaze = np.array(maze).astype(np.uint8)
    showmaze *= 255
    for colorpath in path:
        showmaze[colorpath[0]][colorpath[1]] = 50
    showmaze[start[0]][start[1]] = 180
    showmaze[end[0]][end[1]] = 180
    showmaze = cv2.resize(showmaze, None, fx=7, fy=7, interpolation=cv2.INTER_NEAREST)
    cv2.imshow('Sample A* algorithm run with distance cost', showmaze)
    cv2.waitKey(0)


if __name__ == '__main__':
    main()
