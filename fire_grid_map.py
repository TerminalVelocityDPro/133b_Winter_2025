#!/usr/bin/env python3
#
#   hw5_solution.py probnum    with probnum being: 1a, 1b, 2, 3a, 3b, 4a, 4b
#
#   Homework 5 code to localize a robot in a grid.  The optional part
#   number ('1a', '1b', '2', '3a', '3b', '4a', '4b') determines the
#   settings to match the problem.
#
import numpy as np
import sys

from hw5_utilities import Visualization, Robot
import random
from dstar import Node, Planner

# We first define some helper functions for the code to run

# Compute where the robot is to set current position
def computeCurrentNode(robot,nodes):
    for node in nodes:
        if node.row == robot.row and node.col == robot.col:
            current = node
            return current
        
def checkObstacle(obstacles, node):
    for obstacle in obstacles:
        if node.row == obstacle[0] and node.col == obstacle[1]:
            return True
        else:
            continue
    return False

def setObstacle(node):
    node.type = 'obstacle'
    walls[node.row, node.col] = 1.0  # Mark as a wall/obstacle
                
    # Remove the obstacle node from its neighbors' neighbor lists
    for neighbor in node.neighbors:
        if node in neighbor.neighbors:
            neighbor.neighbors.remove(node)
    node.neighbors.clear()  # Clear the obstacle's neighbors
    
def clearObstacle(node, nodes):
    clearObstacle = False
    row, col = node.row, node.col
    # Only connect to adjacent nodes (single-step moves)
    for drow, dcol in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
        nrow, ncol = row + drow, col + dcol
        # Check if the neighbor is within bounds and not a wall
        if 0 <= nrow < rows and 0 <= ncol < cols and not walls[nrow, ncol]:
            # Find the neighbor node
            neighbor = next((n for n in nodes if n.row == nrow and n.col == ncol), None)
            if neighbor:
                if neighbor.type == 'obstacle' and not checkObstacle(neighbor):
                    neighbor.type = 'clear'
                    node.neighbors.append(neighbor)
                    clearObstacle = True
    return clearObstacle
            
def setFireNodes(goal_mark, nodes):
    for node in nodes: 
        if node.row == goal_mark[0] and node.col == goal_mark[1]:
            node.type = 'fire'
            print(f'Fire set successfully at: ({node.row}, {node.col})')
            walls[node.row, node.col] = 0

def setDogNodes(dog_mark, nodes):
    for node in nodes:
        if node.row == dog_mark[0] and node.col == dog_mark[1]:
            node.type = 'dog'
            print(f'Dog has been set at ({node.row}, {node.col})')
            walls[node.row, node.col] = 0

def connectNodes(nodes, walls):
    rows = len(walls)
    cols = len(walls[0]) if rows > 0 else 0

    for node in nodes: 
        row, col = node.row, node.col

        # Only connect to adjacent nodes (single-step moves)
        for drow, dcol in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            nrow, ncol = row + drow, col + dcol
            # Check if the neighbor is within bounds and not a wall
            if 0 <= nrow < rows and 0 <= ncol < cols and not walls[nrow, ncol]:
                # Find the neighbor node
                neighbor = next((n for n in nodes if n.row == nrow and n.col == ncol), None)
                if neighbor:
                    node.neighbors.append(neighbor)
                    # Ensure the delta values (drow, dcol) are valid (between -1, 0, or 1).

def getValidDelta(drow, dcol):
    # Clamp drow and dcol to valid range [-1, 1]
    return max(-1, min(1, drow)), max(-1, min(1, dcol))

def getValidMove(robot, drow, dcol):
    # Check for walls/obstacles at the intended position
    if walls[robot.row + drow, robot.col + dcol] == 1:
        print(f"Collision detected at ({robot.row + drow}, {robot.col + dcol}). Skipping move.")
        return 0, 0  # Skip move if the target position is a wall

    return getValidDelta(drow, dcol)

def setUpObstacles(walls):
    print("SETTING UP OBSTACLES")
    obstacles = set()
    while len(obstacles) < 300:
        pos = (random.randint(1, rows - 2), random.randint(1, cols - 2))
        if walls[pos[0], pos[1]] == 0:  # Ensure it's not inside a wall
            obstacles.add(pos)
    obstacles = list(obstacles)
    return obstacles

def setGoal(nodes):
    goal = None
    for node in nodes:
        if node.type == 'dog':
            goal = node
            break
        elif node.type == 'fire':
            goal = node
    if goal.type == 'dog':
        print(f'Found dog node at: ({node.row}, {node.col})')
    elif goal.type == 'fire':
        print(f'Found fire node at: ({node.row}, {node.col})')
    elif goal is None:
        print("Error: No fire or dog node found! Exiting.")
        return
    return goal

# Now, we set up the visualization
#
# Define the Walls
#
w = ['xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx',
     'x                                               x',
     'x                                               x',
     'x                                               x',
     'x                                               x',
     'x                                               x',
     'x                                               x',
     'x                                               x',
     'x                                               x',
     'x                                               x',
     'x                                               x',
     'x                                               x',
     'x                                               x',
     'x                                               x',
     'x                                               x',
     'x                                               x',
     'x                                               x',
     'x                                               x',
     'x                                               x',
     'x                                               x',
     'x                                               x',
     'x                                               x',
     'x                                               x',
     'x                                               x',
     'xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx']

walls = np.array([[1.0*(c == 'x') for c in s] for s in w])
# print(walls)

rows  = np.size(walls, axis=0)
cols  = np.size(walls, axis=1)

# Ensure unique and valid obstacle placement
obstacles = setUpObstacles(walls)

# Ensure goal is not inside a wall or in obstacles
goal_mark = (random.randint(1, rows - 2), random.randint(1, cols - 2))
while goal_mark in obstacles or walls[goal_mark[0], goal_mark[1]] == 1:
    goal_mark = (random.randint(1, rows - 2), random.randint(1, cols - 2))

# Ensure goal is not inside a wall or in obstacles
dog_mark = (random.randint(1, rows - 2), random.randint(1, cols - 2))
while dog_mark in obstacles or walls[dog_mark[0], dog_mark[1]] == 1:
    dog_mark = (random.randint(1, rows - 2), random.randint(1, cols - 2))

# 
#
#  Main Code
#
def main():
    global obstacles
    robot=Robot(walls)

    # Initialize the figure.
    visual = Visualization(walls, robot, obstacles, goal_mark, dog_mark)
    input("The empty grid")
    
    nodes = []
    for row in range(rows):
        for col in range(cols):
            # Create a node per space, except only color walls black.
            if w[row][col] != 'x':
                nodes.append(Node(row, col))
    # Connect nodes to their neighbors
    connectNodes(nodes, walls)
    # get starting position
    start = computeCurrentNode(robot,nodes)
    # and find nearest fire to set as goal node
    setFireNodes(goal_mark, nodes)
    setDogNodes(dog_mark, nodes)
    
    goal = setGoal(nodes)
    
    # Plan a path to the first goal (the dog)
    planner = Planner(start, goal, nodes)
    planner.path = planner.computePath()

    if not planner.path:
        print("Error: No valid path to the dog found! Exiting.")
     
    # Loop continually.
    while True:
        # Show the current map.  Also show the position of the robot
        visual.Show(np.ones(np.shape(walls)), markRobot=True)
        
        # check if we found the dog
        if planner.current.row == planner.goal.row and planner.current.col == planner.goal.col:
            if planner.goal.type == 'dog':    
                print("\n**********************************************")
                print("* You found the dog! Yay! 🎉🐕🎉              *")
                print("* Now, let's find the fire! 🚒🚒🚒              *")
                print("**********************************************")
                # clear dog goal node
                planner.goal.type = 'clear'
                walls[planner.goal.row, planner.goal.col] = 0
                # and set goal to fire
                planner.goal = setGoal(nodes)
                # then generate new path
                planner.path = planner.computePath()
                visual.Show(np.ones(np.shape(walls)), markRobot=True) 

            elif planner.goal.type == 'fire':
                print("\n**********************************************")
                print("* You found the fire! Yay! 🎉🔥🎉               *")
                print("* Mission accomplished! 🚒🚒🚒                   *")
                print("**********************************************")
                return
        
        # Check for obstacles and recalculate path if necessary.
        if checkObstacle(obstacles,planner.path[0]):
            setObstacle(planner.path[0])
            planner.path = planner.computePath()
            if not planner.path:
                print("Error: No valid path found after obstacle adjustment. Exiting.")
                return
        if clearObstacle(planner.path[0],nodes):
            planner.path = planner.computePath()

        if not planner.path:
            print("No path available. Exiting.")
            return

        # Get the next node in the path
        next_node = planner.path.pop(0)
        print(f"Moving from ({planner.current.row}, {planner.current.col}) to ({next_node.row}, {next_node.col})")

        # Calculate the total delta required to reach the next node
        total_drow = next_node.row - planner.current.row
        total_dcol = next_node.col - planner.current.col
                
        # Move the robot
        robot.Command(total_drow, total_dcol)
        planner.current = computeCurrentNode(robot, nodes)
        print(f'Move completed. Current position: ({planner.current.row}, {planner.current.col})')
        input("Press Enter to move to the next step...")
        visual.SetObstacles()
        obstacles = visual.obstacles

if __name__== "__main__":
    main()