from graphics import *
from math import *

## A program to read in an ATSP solution file and output it in a visual form.
## Author: Bryan McKenney

# Initialize variables
size = 0
distances = [[]]
tour = []
fillMode = "none"
row = 0
nodes = []
totalDis = 0

# Read in data from solution file
with open("solution.txt", "r") as file:
  for line in file:
    # Strip newline from line and split by spaces
    line = line.strip().split(" ")

    if (fillMode == "none"):
        if (line[0] == "SIZE:"): # Set size and initialize arrays
            size = int(line[1])
            distances = [0 for i in range(size)]
            nodes = [None for i in range(size)]
        elif (line[0] == "PROBLEM"): # Start filling in distances
            fillMode = "distances"
        elif (line[0] == "SOLUTION"): # Start filling in tour
            fillMode = "tour"
    elif (fillMode == "distances"): # Read in distance matrix
        distances[row] = [float(numStr) for numStr in line] # strings -> floats
        row += 1

        # Done reading in distance matrix
        if (row >= size):
            fillMode = "none"
    else: # Read in solution tour
        tour = [int(numStr) for numStr in line] # strings -> ints
        fillMode = "none"

# Create window for drawing
win = GraphWin("ATSP Visualization", 500, 500)
center = Point(win.getWidth() / 2, win.getHeight() / 2)
angleMod = 360 / size

# Draw nodes
for i in range(size):
  # Determine x and y coordinates of this node
  x = center.getX() + 200 * cos(radians(-90 + angleMod * i))
  y = center.getY() + 200 * sin(radians(-90 + angleMod * i))

  # Create and draw the node circle
  c = Circle(Point(x, y), 20)
  c.setWidth(2)
  c.setFill("white")
  c.draw(win)

  # Add the center of this circle to the nodes array
  nodes[i] = c

  # Add text for node number in the center of the circle
  t = Text(c.getCenter(), i)
  t.setSize(24)
  t.setTextColor("steelblue")
  t.draw(win)

# Draw arrows
for i in range(1, len(tour)):
  # Get node number for the last and current nodes in the tour
  lastNode = tour[i - 1]
  currNode = tour[i]

  # Add distance between these nodes to the total
  totalDis += distances[lastNode][currNode]

  # Draw arrow between those nodes
  l = Line(nodes[lastNode].getCenter(), nodes[currNode].getCenter())
  l.setArrow("last")
  l.draw(win)

# Draw tour distance
t = Text(Point(60, 20), "Distance: " + str(totalDis))
t.draw(win)

# Close window on click
win.getMouse()
win.close()
