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
print(walls)
obstacles = [[(10,10),
              (10,11),
              (11,11),
              (11,10),
              (11,12),
              (12,10),
              (12,9),
              (12,11),
              (13,9),
              (12,8)],
             [(17,15),
              (17,16),
              (17,17),
              (17,18),
              (17,14),
              (16,15),
              (16,16),
              (16,17),
              (16,18)]]

rows  = np.size(walls, axis=0)
cols  = np.size(walls, axis=1)

#
#  Prediction
#
#    bel         Grid of probabilities (current belief)
#    drow, dcol  Delta in row/col
#    pCmdUsed    Modeled probability of command executing
#    prd         Grid of probabilities (prediction)
#
def computePrediction(bel, drow, dcol, pCmdUsed = 1, kidnap = False):
    # Prepare an empty prediction grid.
    prd = np.zeros((rows,cols))

    # Iterate over/determine where "a belief will go".
    for row in range(1, rows-1):
        for col in range(1, cols-1):
            # Try to shift by the given delta.
            if not walls[row+drow, col+dcol]:
                # If the move is legal, shift the belief over.
                prd[row+drow, col+dcol] +=        pCmdUsed  * bel[row, col]
                prd[row,      col]      += (1.0 - pCmdUsed) * bel[row, col]
            else:
                # If the move is illegal, keep the belief where it is.
                prd[row,      col]      +=                   bel[row, col]

    # If we allow kidnapping, add a small probability (say 1%) everywhere.
    if kidnap:
        uniform = (1.0 - walls) / np.sum(1.0 - walls)
        prd = 0.99 * prd + 0.01 * uniform

    # Return the prediction grid
    return prd


#
#  Measurement Update (Correction)
#
#    prior       Grid of prior probabilities (belief)
#    probSensor  Grid of probabilities that (sensor==True)
#    sensor      Value of sensor
#    post        Grid of posterior probabilities (updated belief)
#
def updateBelief(prior, probSensor, sensor):
    # Update the prior probability based on the sensor reading, which
    # can be one of two cases: (sensor==True) or (sensor==False)
    if (sensor):
        # Use the probability of the sensor reading True.
        post = prior * probSensor
    else:
        # Use the probability of the sensor reading False.
        post = prior * (1.0 - probSensor)

    # Normalize.
    s = np.sum(post)
    if (s == 0.0):
        print("LOST ALL BELIEF - THIS SHOULD NOT HAPPEN!!!!")
    else:
        post = (1.0/s) * post
    return post


#
#  Pre-compute the Sensor Probability Grid
#
#    drow, dcol   Direction in row/col
#    pSenDist     List of probabilities that sensor triggers at dist=(index+1)
#    prob         Grid of probabilities that (sensor==True)
#
def precomputeSensorProbability(drow, dcol, pSenDist = [1.0]):
    # Assume the probability is zero, unless updated below.
    prob = np.zeros((rows, cols))

    # Pre-compute the probability for each grid element, knowing the
    # walls and the direction of the sensor.
    for row in range(1, rows-1):
        for col in range(1, cols-1):
            # Check for a wall at multiple steps in the given delta direction.
            for k in range(len(pSenDist)):
                if walls[row + drow*(k+1), col + dcol*(k+1)]:
                    prob[row, col] = pSenDist[k]
                    break

    # Return the computed grid.
    return prob


def computeCurrentNode(robot,nodes):
    for node in nodes:
        if node.row == robot.row and node.col == robot.col:
            current = node
            return current
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


    if   part == '1a':
        robot=Robot(walls)
    elif part == 'play':
        robot=Robot(walls, pSensor=[0.9,0.6,0.3], pCommand=0.8)

    # Initialize the figure.
    visual = Visualization(walls, robot, obstacles)
    input("The empty grid")
    
    nodes  = []
    for row in range(rows):
        for col in range(cols):
            # Create a node per space, except only color walls black.
            if w[row][col] != '#':
                nodes.append(Node(row, col))
    # get starting position
    start = computeCurrentNode(robot,nodes)
    # and find nearest fire to set as goal node
    fire_distance = np.inf
    for node in nodes:
        if node.type == 'fire':
            temp_dist = node.distance(start)
            if temp_dist < fire_distance:
                fire_distance = temp_dist
                goal = node
    
    planner = Planner(start, goal)
    
    # compute initial path, with no knowledge of fires or obstacles
    path = planner.computePath()

    # Start with a uniform belief grid.
    bel = (1.0 - walls) / np.sum(1.0 - walls)

    # Loop continually.
    while True:
        # Show the current belief.  Also show the actual position.
        visual.Show(np.ones(np.shape(walls)), markRobot=True)

        # Get the command key to determine the direction.
        # while True:
        #    key = input("Cmd (q=quit, i=up, m=down, j=left, k=right) ?")
        #    if   (key == 'q'):  return
        #    elif (key == 'i'):  (drow, dcol) = (-1,  0) ; break
        #    elif (key == 'm'):  (drow, dcol) = ( 1,  0) ; break
        #    elif (key == 'j'):  (drow, dcol) = ( 0, -1) ; break
        #    elif (key == 'k'):  (drow, dcol) = ( 0,  1) ; break

        # check for obstacle
        while path[0].type == 'obstacle':
            path[0].type = np.inf
            path = planner.computePath()
        
        # and get next move
        next_node = path.pop(0)
        drow = next_node.row - planner.current.row
        dcol = next_node.col - planner.current.col

        # Move the robot in the simulation.
        robot.Command(drow, dcol)
        planner.current = computeCurrentNode(robot,nodes)

if __name__== "__main__":
    main()
