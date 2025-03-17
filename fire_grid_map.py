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


#
#  Define the Walls
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
obstacles = set()
while len(obstacles) < 30:
    pos = (random.randint(1, rows - 2), random.randint(1, cols - 2))
    if walls[pos[0], pos[1]] == 0:  # Ensure it's not inside a wall
        obstacles.add(pos)
obstacles = list(obstacles)

# Ensure goal is not inside a wall or in obstacles
goal_mark = (random.randint(1, rows - 2), random.randint(1, cols - 2))
while goal_mark in obstacles or walls[goal_mark[0], goal_mark[1]] == 1:
    goal_mark = (random.randint(1, rows - 2), random.randint(1, cols - 2))

# compute where the robot is to set current position
def computeCurrentNode(robot,nodes):
    for node in nodes:
        if node.row == robot.row and node.col == robot.col:
            current = node
            return current
        
def setObstacleNodes(obstacles, nodes):
    for node in nodes:
        for obstacle in obstacles:
            if node.row == obstacle[0] and node.col == obstacle[1]:
                node.type = 'obstacle'
                walls[node.row, node.col] = 1.0  # Mark as a wall/obstacle
                
                # Remove the obstacle node from its neighbors' neighbor lists
                for neighbor in node.neighbors:
                    if node in neighbor.neighbors:
                        neighbor.neighbors.remove(node)
                node.neighbors.clear()  # Clear the obstacle's neighbors
                break
            
def setFireNodes(goal_mark, nodes):
    for node in nodes: 
        if node.row == goal_mark[0] and node.col == goal_mark[1]:
            node.type = 'fire'
            print(f'Fire set successfully at: ({node.row}, {node.col})')
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

# 
#
#  Main Code
#
def main():
    # Check the command line to decide which part we want.
    if len(sys.argv) > 1:
        part = sys.argv[1]
    else:
        part = 'play'
    print("Running for part: %s" % part)


    if part == '1a':
        robot=Robot(walls)
    elif part == 'play':
        robot=Robot(walls, pSensor=[0.9,0.6,0.3], pCommand=0.8)

    # Initialize the figure.
    visual = Visualization(walls, robot, obstacles, goal_mark)
    input("The empty grid")
    
    nodes = []
    for row in range(rows):
        for col in range(cols):
            # Create a node per space, except only color walls black.
            if w[row][col] != '#':
                nodes.append(Node(row, col))
    # Connect nodes to their neighbors
    connectNodes(nodes, walls)
    # get starting position
    start = computeCurrentNode(robot,nodes)
    # and find nearest fire to set as goal node
    setFireNodes(goal_mark, nodes)
    setObstacleNodes(obstacles,nodes)
    fire_distance = np.inf
    goal = None
    for node in nodes:
        if node.type == 'fire':
            print(f'Found fire node at: ({node.row}, {node.col})')
            temp_dist = node.distance(start)
            if temp_dist < fire_distance:
                fire_distance = temp_dist
                goal = node
                
    if goal is None: 
        print("Error: No fire node found! Exiting.")
        return
     
    planner = Planner(start, goal, walls, nodes)
    
    # compute initial path, with no knowledge of fires or obstacles
    planner.path = planner.computePath()
    
    if not planner.path:
        print("Error: No valid path found! Trying alternative paths...")
        alternative_path = planner.computePath()  # Try recalculating once more
        if not alternative_path:
            print("Critical Error: No valid paths exist. Exiting.")
            return
        else:
            planner.path = alternative_path

    # Start with a uniform belief grid.
    bel = (1.0 - walls) / np.sum(1.0 - walls)

    # Loop continually.
    while True:
        # Show the current belief.  Also show the actual position.
        visual.Show(np.ones(np.shape(walls)), markRobot=True)

        if planner.current.row == goal.row and planner.current.col == goal.col:
            print("\n**********************************************")
            print("* You found the fire! Yay! 🎉🔥🎉               *")
            print("* Mission accomplished! 🚒🚒🚒                   *")
            print("**********************************************")
            return
        
        # Check for obstacles and recalculate path if necessary.
        while planner.path and planner.path[0].type == 'obstacle':
            print(f"Encountered obstacle at {planner.path[0].row}, {planner.path[0].col}, recalculating path...")
            planner.path = planner.computePath()
            if not planner.path:
                print("Error: No valid path found after obstacle adjustment. Exiting.")
                return

        if not planner.path:
            print("No path available. Exiting.")
            return

        # Get the next node in the path
        next_node = planner.path.pop(0)
        print(f"Moving from ({planner.current.row}, {planner.current.col}) to ({next_node.row}, {next_node.col})")

        # Calculate the total delta required to reach the next node
        total_drow = next_node.row - planner.current.row
        total_dcol = next_node.col - planner.current.col

        # Move the robot one step at a time
        while total_drow != 0 or total_dcol != 0:
            # Calculate the single-step delta
            step_drow = getValidDelta(total_drow, 0)[0]  # Clamp to [-1, 1]
            step_dcol = getValidDelta(0, total_dcol)[1]  # Clamp to [-1, 1]

            # Check for walls/obstacles at the intended position
            if walls[planner.current.row + step_drow, planner.current.col + step_dcol] == 1:
                print(f"Collision detected at ({planner.current.row + step_drow}, {planner.current.col + step_dcol}). Recalculating path...")
                
                # Mark the collision point as an obstacle
                obstacle_node = Node(planner.current.row + step_drow, planner.current.col + step_dcol)
                obstacle_node.type = 'obstacle'
                setObstacleNodes([(obstacle_node.row, obstacle_node.col)], nodes)
                
                # Recalculate the path
                planner.path = planner.computePath()
                if not planner.path:
                    print("Error: No valid path found after collision. Exiting.")
                    return
                
                # Restart movement with the new path
                break
                
            # Move the robot
            robot.Command(step_drow, step_dcol)
            planner.current = computeCurrentNode(robot, nodes)

            # Update the remaining delta
            total_drow = next_node.row - planner.current.row
            total_dcol = next_node.col - planner.current.col
            print(f"Step completed. Current position: ({planner.current.row}, {planner.current.col}), Remaining delta: ({total_drow}, {total_dcol})")

        print(f'Move completed. Current position: ({planner.current.row}, {planner.current.col})')
        input("Press Enter to move to the next step...")

if __name__== "__main__":
    main()