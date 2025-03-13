# define D* Algorithm using Node class from sets
import numpy as np
import math
import bisect
from math import inf
    
class Node:
    # Initialization
    def __init__(self, row, col):
        # Save the matching state.
        self.row = row
        self.col = col

        # Clear the list of neighbors (used for the full graph).
        self.neighbors = []

        # Clear the parent (used for the search tree), as well as the
        # actual cost to reach (via the parent).
        self.parent = None      # No parent
        self.child = None       # and no child
        # introduce variable to store the cost to reach the node so far
        # (excluding the estimated cost)
        self.cost2Reach = inf   # Unable to reach = infinite cost
        # now, we treat cost as the 'total cost' including the estimated cost
        # to go and the cost to reach the node so far
        self.cost = inf         # Initialize total cost as infinite

        # State of the node during the search algorithm.
        self.seen = False
        self.done = False


    # Define the Chebyshev distance to another node.
    def distance(self, other):
        return max(abs(self.row - other.row),abs(self.col - other.col))

    # Define the "less-than" to enable sorting by cost.
    def __lt__(self, other):
        return self.cost < other.cost


    # Print (for debugging).
    def __str__(self):
        return("(%2d,%2d)" % (self.row, self.col))
    def __repr__(self):
        return("<Node %s, %7s, cost %f>" %
               (str(self),
                "done" if self.done else "seen" if self.seen else "unknown",
                self.cost))


#
#   Search/Planner Algorithm
#
#   This is the core algorithm.  It builds a search tree inside the
#   node graph, transfering nodes from air (not seen) to leaf (seen,
#   but not done) to trunk (done).
#
# Run the planner.
def computePath(current, goal):
    # Use the goal node to initialize the on-deck queue *note D* Lite starts
    # at the goal instead of the start node
    goal.seen = True
    goal.cost = 0
    goal.cost2Reach = 0
    goal.child = None
    goal.parent = None
    onDeck = [goal]

    # Continually expand/build the search tree.
    print("Starting the processing...")
    while True:
        # Check that priority queue isn't empty
        if not (len(onDeck) > 0):
            return None

        # Grab the next state (first on the storted on-deck list).
        node = onDeck.pop(0)

        ####################
        
        # stop search if path to start is found
        if node == current:
            break
        
        for neighbor in node.neighbors:
            # check that neighbor has not already been done
            if not neighbor.done:
                # compute cost to get to neighbor from current node
                tempcost2Reach = node.cost2Reach + 1
                # compute estimated cost to reach goal from current node
                cost2Go = neighbor.distance(goal)
                # and create estimated total cost
                totalCost = tempcost2Reach + cost2Go
                # check if this is the optimal cost and update accordingly
                if totalCost < neighbor.cost:
                    neighbor.cost2Reach = tempcost2Reach
                    neighbor.cost = totalCost
                    # update parent and child for finding path later
                    neighbor.child = node
                    node.parent = neighbor
                    # make sure to update the neighbor within onDeck
                    if neighbor.seen:
                        onDeck.remove(neighbor)
                    # or, mark the neighbor as seen for the first time
                    else:
                        neighbor.seen = True
                    # lastly, insert the neighbor into the list
                    bisect.insort(onDeck,neighbor)
        # mark node as done to avoid repeats
        node.done = True
    
    # now, construct path to return by working backwards from the goal
    path = []
    brick = goal
    while True:
        bisect.insort(path, brick)
        if brick.parent:
            brick = brick.parent
        else:
            return path