# define D* Algorithm using Node class from sets
import numpy as np
import math
import bisect
from numpy import inf
    
class Node:
    # Initialization
    def __init__(self, row, col):
        # Save the matching state.
        self.row = row
        self.col = col
        # initialize all nodes as clear
        self.type = 'clear' # other options are 'fire', 'obstacle', 'dog'

        # Clear the list of neighbors (used for the full graph).
        self.neighbors = []

        # Clear the parent (used for the search tree), as well as the
        # actual cost to reach (via the parent).
        self.parent = None      # No parent
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
        return int(max(abs(self.row - other.row),abs(self.col - other.col)))

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

class Planner:
    
    def __init__(self, current, goal, nodes):
        self.path = None
        self.current = current
        self.goal = goal
        self.nodes = nodes
    
    def initNodes(self):
        for node in self.nodes:
            node.seen = False
            node.done = False
            node.cost = inf
            node.cost2Reach = inf
            node.parent = None
        
    # Run the planner.
    def computePath(self):
        self.initNodes()
        # Use the goal node to initialize the on-deck queue *note D* Lite starts
        # at the goal instead of the start node
        self.goal.seen = True
        self.goal.cost = self.goal.distance(self.current)
        self.goal.cost2Reach = 0
        self.goal.parent = None
        onDeck = [self.goal]

        print("Deck")
        print(onDeck)

        # Continually expand/build the search tree.
        print("Starting the processing...")
        while True:
            # print(self.path)
            # Check that priority queue isn't empty
            if not onDeck:
                print("NO PRIORITY QUEUE")
                break
                # return None

            # Grab the next state (first on the storted on-deck list).
            print(f"THE CURRENT NODE IS {self.current.row} and {self.current.col}")
            print(f"THE GOAL NODE IS {self.goal.row} and {self.goal.col}")
            print(f"the queue is {onDeck}")
            node = onDeck.pop(0)
            print(f"the node is {node}")
            print(f"the node's neighbors is {node.neighbors}")

            ####################
            
            # stop search if path to start is found
            if node == self.current:
                print("SEARCH STOPPED")
                break

            for neighbor in node.neighbors:
                print("THE NEIGHBOR IS ")
                print(neighbor)
                print(neighbor.cost)
                # check that neighbor has not already been done
                if not neighbor.done:
                    # determines if move is diagonal
                    drow = abs(neighbor.row - node.row)
                    dcol = abs(neighbor.col - node.col)
                    move_cost = math.sqrt(2) if drow == 1 and dcol == 1 else 1
                    
                    # compute cost to get to neighbor from current node
                    tempcost2Reach = int(node.cost2Reach + move_cost)
                    # compute estimated cost to reach goal from current node
                    cost2Go = neighbor.distance(self.current)
                    # and create estimated total cost
                    totalCost = tempcost2Reach + cost2Go
                    # check if this is the optimal cost and update accordingly
                    print("THE TOTAL COST IS ")
                    print(totalCost)
                    if totalCost <= neighbor.cost:
                        neighbor.cost2Reach = tempcost2Reach
                        neighbor.cost = totalCost
                        # update parent for finding path later
                        neighbor.parent = node
                        # make sure to update the neighbor within onDeck
                        if neighbor.seen:
                            onDeck.remove(neighbor)
                        # or, mark the neighbor as seen for the first time
                        else:
                            neighbor.seen = True
                        # lastly, insert the neighbor into the list
                        bisect.insort(onDeck,neighbor)
                else:
                    print("DONE")
            # mark node as done to avoid repeats
            node.done = True
        
        # now, construct path to return by working backwards from the goal
        self.path = []
        brick = self.current.parent
        # print("BRICK's PARENT")
        # print(brick.parent)
        while brick:
            self.path.append(brick) 
            brick = brick.parent

        print("THE COMPUTED PATH IS:")
        print(self.path)
        return self.path