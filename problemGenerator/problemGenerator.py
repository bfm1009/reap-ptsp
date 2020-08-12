from random import *
from math import floor

'''
Program that generates random PTSP files.
Author: Bryan McKenney
'''

# Constants
NUM_PROBS = 1000
MIN_SIZE = 5
MAX_SIZE = 10
SCALE = 10
MIN_WAYPOINTS = 2
MAX_WAYPOINTS = 6
MIN_OBSTACLE_PERCENT = 5
MAX_OBSTACLE_PERCENT = 50
RADIUS = 3

# Global variables
width = 0
height = 0
initX = 0
initY = 0
numWaypoints = 0
waypoints = []
map = []
i = 0
waypointsFound = 0


# Returns a random point within the map
def randPoint():
    x = randint(0, width - 1)
    y = randint(0, height - 1)
    return [x, y]


# Returns if a point is already occupied by an obstacle, waypoint, or the vehicle
def occupied(pt):
    x = pt[0]
    y = pt[1]

    # This point is where the vehicle starts
    if (x == initX and y == initY):
        return True

    # This point is on a waypoint or an obstacle
    if (map[x][y] == "@" or map[x][y] == "#"):
        return True

    # This point is in an empty cell
    return False


# Finds a point that is not occupied by something already
def unoccupiedPoint():
    pt = randPoint()

    while (occupied(pt)):
        pt = randPoint()

    return pt


# Validate that the problem can be solved (i.e. all waypoints can be reached from the start position)
def validateProblem():
    global waypointsFound
    waypointsFound = 0
    return checkCell(initX, initY)


# Check a cell for if it's a waypoint and call this on all open adjacent cells
def checkCell(x, y):
    global waypointsFound

    # Check if cell is within the map
    if (x >= 0 and x < len(map) and y >= 0 and y < len(map[0])):
        # Check cell
        if (map[x][y] == "#" or map[x][y] == "_"): # Stop if cell is an obstacle or already visited
            return False
        elif (map[x][y] == "@"):
            waypointsFound += 1
        
        # Mark cell as being visited
        map[x][y] = "_"
    else:
        return False

    # Check adjacent cells
    checkCell(x + 1, y)
    checkCell(x - 1, y)
    checkCell(x, y + 1)
    checkCell(x, y - 1)

    return (waypointsFound == numWaypoints)


# Generate the problem files
while (i < NUM_PROBS):
    # Randomly generate problem
    width = randint(MIN_SIZE, MAX_SIZE)
    height = randint(MIN_SIZE, MAX_SIZE)
    numWaypoints = randint(MIN_WAYPOINTS, MAX_WAYPOINTS)
    initDir = randrange(0, 359)
    initX = randint(0, width)
    initY = randint(0, height)
    obstaclePercent = randint(MIN_OBSTACLE_PERCENT, MAX_OBSTACLE_PERCENT)
    numObstacles = floor(width * height * (obstaclePercent / 100))
    map = [["." for k in range(height)] for j in range(width)]

    for j in range(numWaypoints):
        pt = unoccupiedPoint()
        x = pt[0]
        y = pt[1]
        map[x][y] = "@"
        waypoints.append(pt)

    for j in range(numObstacles):
        pt = unoccupiedPoint()
        x = pt[0]
        y = pt[1]
        map[x][y] = "#"

    # Test problem to see if it is possible to hit all the waypoints
    valid = validateProblem()

    # Write problem to file if it is valid
    if (valid):
        with open(f"problems/problem{i}.ptsp", "w+") as file:
            file.write(f"WORLD_DIMENSIONS: {width * SCALE} {height * SCALE}\n")
            file.write(f"NUM_WAYPOINTS: {numWaypoints}\n")
            file.write(f"INITIAL_DIR: {initDir}\n")
            file.write(f"INITIAL_POS: {initX * SCALE + SCALE / 2} {initY * SCALE + SCALE / 2}\n")
            file.write("MAP\n")
            
            for y in range(len(map[0]) - 1, -1, -1):
                for x in range(len(map)):
                    if (map[x][y] == "."): # Don't display places where vehicle couldn't reach
                        file.write("_")
                    else:
                        file.write(map[x][y])
                file.write("\n")

            file.write("WAYPOINTS\n")

            for j in range(numWaypoints):
                file.write(f"{j + 1}\t{waypoints[j][0] * SCALE + SCALE / 2} {waypoints[j][1] * SCALE + SCALE / 2} {RADIUS}\n")
        
        i += 1

    # Reset waypoints for next iteration
    waypoints = []