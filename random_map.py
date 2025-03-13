#!/usr/bin/env python3
#
#   rrttriangles.py
#
#   Use RRT to find a path around triangular obstacles.
#
#   This is skeleton code. Please fix, especially where marked by "FIXME"!
#
import matplotlib.pyplot as plt
import numpy as np
import random
import time

from math               import pi, sin, cos, atan2, sqrt, ceil, dist
from scipy.spatial      import KDTree
from shapely.geometry   import Point, LineString, Polygon, MultiPolygon
from shapely.prepared   import prep
from shapely.plotting   import plot_polygon


######################################################################
#
#   Parameters
#
#   Define the step size.  Also set the maximum number of nodes.
#
DSTEP = 0.25

# Maximum number of steps (attempts) or nodes (successful steps).
SMAX = 50000
NMAX = 1500

# TOTAL NUM OF TICKS
TICK_NUM = 24


######################################################################
#
#   World Definitions
#
#   List of obstacles/objects as well as the start/goal.
#
(xmin, xmax) = (0, 10)
(ymin, ymax) = (0, 12)

# TRIANGLE DEFINITION
# first pair is first vertex
# second, third, so on...
# fourth = first
triangle1 = Polygon([[6, 3], [2,  3], [2, 4], [6, 3]])  
triangle2 = Polygon([[4, 5], [6,  6], [4, 7], [4, 5]])
triangle3 = Polygon([[8, 4], [8,  7], [6, 5], [8, 4]])
triangle4 = Polygon([[2, 9], [5, 10], [5, 8], [2, 9]])
triangle5 = Polygon([[0, 12], [4, 11], [2, 10], [0, 12]])


# Collect all the triangle and prepare (for faster checking).
triangles = prep(MultiPolygon([
    triangle1,
    triangle2,
    triangle3,
    triangle4,
    triangle5]))

current_triangles = []
current_triangles_polygon = None

triangle_list = [triangle1, triangle2, triangle3, triangle4, triangle5]

# Define the start/goal states (x, y, theta)
(xstart, ystart) = (5, 1)
(xgoal,  ygoal)  = (4, 11)


######################################################################
#
#   Utilities: Visualization
#
# Visualization Class
class Visualization:
    def __init__(self):
        self.tick = 0
        self.show_triangle_tick = 0
        self.used_triangles = [False for n in range(len(triangle_list))]
        # Clear the current, or create a new figure.
        plt.clf()

        # Create a new axes, enable the grid, and set axis limits.
        plt.axes()
        plt.grid(True)
        plt.gca().axis('on')
        plt.gca().set_xlim(xmin, xmax)
        plt.gca().set_ylim(ymin, ymax)
        plt.gca().set_aspect('equal')

        # Show the triangles.
        # for poly in triangles.context.geoms:
        #     plt.plot(*poly.exterior.xy, 'k-', linewidth=2)

        # Show.
        self.show()

    def show(self, text = ''):
        # Show the plot.
        plt.pause(0.001)
        # If text is specified, print and wait for confirmation.
        if len(text)>0:
            input(text + ' (hit return to continue)')
    
    def drawTriangles(self):
        for poly in triangles.context.geoms:
            plt.plot(*poly.exterior.xy, 'k-', linewidth=2)

    def drawTriangle(self):
        current_selection = -1
        current_status = False

        ## CHECKING IF ALL HAVE BEEN USED BEFORE
        all_used = True 
        for i in range(len(triangle_list)):
            if not self.used_triangles[i]:
                all_used = False

        if not all_used:
            while current_selection == -1 or not current_status:
                current_selection = random.randint(0, len(self.used_triangles) - 1)
                print("the current selection is")
                print(current_selection)
                print(self.used_triangles)
                print(self.used_triangles[current_selection])
                if self.used_triangles[current_selection] is False:
                    break

            plot_polygon(triangle_list[current_selection])
            self.used_triangles[current_selection] = True
            current_triangles.append(triangle_list[current_selection])
            current_triangles_polygon = prep(MultiPolygon(current_triangles))

    def drawNode(self, node, *args, **kwargs):
        plt.plot(node.x, node.y, *args, **kwargs)

    def drawEdge(self, head, tail, *args, **kwargs):
        plt.plot((head.x, tail.x),
                 (head.y, tail.y), *args, **kwargs)

    def drawPath(self, path, *args, **kwargs):
        for i in range(len(path)-1):
            self.drawEdge(path[i], path[i+1], *args, **kwargs)

    def tick_enter(self):
        input("TICK ")
        print(self.tick)
        self.tick+=1
        if self.tick % self.show_triangle_tick == 0:
            print("HIT")
            print(self.tick)
            self.drawTriangle()

    def set_triangle_visibility_tick(self, tick_val):
        self.show_triangle_tick = tick_val



######################################################################
#
#   Node Definition
#
class Node:
    def __init__(self, x, y):
        # Define a parent (cleared for now).
        self.parent = None

        # Define/remember the state/coordinates (x,y).
        self.x = x
        self.y = y

    ############
    # Utilities:
    # In case we want to print the node.
    def __repr__(self):
        return ("<Point %5.2f,%5.2f>" % (self.x, self.y))

    # Compute/create an intermediate node.  This can be useful if you
    # need to check the local planner by testing intermediate nodes.
    def intermediate(self, other, alpha):
        return Node(self.x + alpha * (other.x - self.x),
                    self.y + alpha * (other.y - self.y))

    # Return a tuple of coordinates, used to compute Euclidean distance.
    def coordinates(self):
        return (self.x, self.y)

    # Compute the relative Euclidean distance to another node.
    def distance(self, other):
        return dist(self.coordinates(), other.coordinates())

    ################
    # Collision functions:
    # Check whether in free space.
    def inFreespace(self):
        if (self.x <= xmin or self.x >= xmax or
            self.y <= ymin or self.y >= ymax):
            return False
        if current_triangles_polygon is None:
            return True
        print(current_triangles)
        return current_triangles_polygon.disjoint(Point(self.coordinates()))
        

    # Check the local planner - whether this connects to another node.
    def connectsTo(self, other):
        line = LineString([self.coordinates(), other.coordinates()])
        return triangles.disjoint(line)


######################################################################
#
#   RRT Functions
#
def rrt(startnode, goalnode, visual):
    # Start the tree with the startnode (set no parent just in case).
    startnode.parent = None
    tree = [startnode]

    # Function to attach a new node to an existing node: attach the
    # parent, add to the tree, and show in the figure.
    def addtotree(oldnode, newnode):
        newnode.parent = oldnode
        tree.append(newnode)
        visual.drawEdge(oldnode, newnode, color='g', linewidth=1)
        visual.show()

    # Loop - keep growing the tree.
    steps = 0
    while True:
        # Determine the target state.

        # PART A
        # targetnode = Node(random.uniform(xmin, xmax), random.uniform(ymin, ymax))

        # PART B
        if random.uniform(0, 1) <= 0.05:
            targetnode = goalnode
        else:
            targetnode = Node(random.uniform(xmin, xmax), random.uniform(ymin, ymax))


        # Directly determine the distances to the target node.
        distances = np.array([node.distance(targetnode) for node in tree])
        index     = np.argmin(distances)
        nearnode  = tree[index]
        d         = distances[index]

        # Determine the next node.
        if d > 0:
            nextnode = nearnode.intermediate(targetnode, DSTEP / d)
        else:
            nextnode = nearnode.intermediate(targetnode, 0)

        # Check whether to attach.
        if nextnode.inFreespace() and nearnode.connectsTo(nextnode):
            addtotree(nearnode, nextnode)

            # If within DSTEP, also try connecting to the goal.  If
            # the connection is made, break the loop to stop growing.
            if goalnode.distance(nextnode) < DSTEP:
                addtotree(nextnode, goalnode)
                break

        # Check whether we should abort - too many steps or nodes.
        steps += 1
        # if (steps >= SMAX) or (len(tree) >= NMAX):
        #     print("Aborted after %d steps and the tree having %d nodes" %
        #           (steps, len(tree)))
        #     return None

    # Build the path.
    path = [goalnode]
    while path[0].parent is not None:
        path.insert(0, path[0].parent)

    # Report and return.
    print("Finished after %d steps and the tree having %d nodes" %
          (steps, len(tree)))

    tree = []
    steps = 0
    return path


# Post process the path.
def PostProcess(path):
    i = 0
    while (i < len(path)-2):
        if path[i].connectsTo(path[i+2]):
            path.pop(i+1)
        else:
            i = i+1


######################################################################
#
#  Main Code
#
def main():
    # Report the parameters.
    print('Running with step size ', DSTEP, ' and up to ', NMAX, ' nodes.')

    # Create the figure.
    visual = Visualization()


    xstart = -1
    ystart = -1
    xgoal = -1
    ygoal = -1

    startnode = Node(xstart, ystart)

    while xstart < 0 or ystart < 0 or not startnode.inFreespace():
        xstart = random.uniform(xmin, xmax)
        ystart = random.uniform(ymin, ymax)
        startnode = Node(xstart, ystart)

    goalnode = Node(xgoal, ygoal)

    while xgoal < 0 or ygoal < 0 or not goalnode.inFreespace():
        xgoal = random.uniform(xmin, xmax)
        ygoal = random.uniform(ymin, ymax)
        goalnode = Node(xgoal, ygoal)

    # Create the start/goal nodes.
    # startnode = Node(xstart, ystart)
    # goalnode  = Node(xgoal,  ygoal)

    # # Show the start/goal nodes.
    visual.drawNode(startnode, color='orange', marker='o')
    visual.drawNode(goalnode,  color='purple', marker='o')
    visual.show("end")

    # PERIODIC SHOWING OF MORE TRIANGLES
    visual.set_triangle_visibility_tick(4)
    # TOTAL NUMBER OF TICKS
    for i in range(TICK_NUM):
        visual.tick_enter()
        path = rrt(startnode, goalnode, visual)
        if not path:
            return
        # visual.drawPath(path, color='r', linewidth=2)
        PostProcess(path)
        visual.drawPath(path, color='b', linewidth=2)
        visual.show()
    visual.show("end")

    # # Run the RRT planner.
    # print("Running RRT...")
    # path = rrt(startnode, goalnode, visual)

    # # If unable to connect, just note before closing.
    # if not path:
    #     visual.show("UNABLE TO FIND A PATH")
    #     return

    # # Show the path.
    # visual.drawPath(path, color='r', linewidth=2)
    # visual.show("Showing the raw path")


    # # Post process the path.
    # PostProcess(path)

    # # Show the post-processed path.
    # visual.drawPath(path, color='b', linewidth=2)
    # visual.show("Showing the post-processed path")


if __name__== "__main__":
    main()
