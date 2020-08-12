import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk, Gdk, GLib
import cairo
from math import *
import sys
import numpy as np

## Program that reads in two files -- a PTSP problem file and a file containing a tree and partial trajectory from DIRT -- and draws them.
## Author: Bryan McKenney

# Global constants
SCALE = 5 # Scale the world size to the screen by this amount
REFLECT_X = cairo.Matrix(1, 0, 0, -1) # Matrix to reflect something over the x-axis
TRI_SIZE = 20 # The size to draw the triangle representing the vehicle
COLOR_MODES = ["grayscale", "neon", "unh"] # The different color modes that the animation can be displayed in

# Global variables
colorMode = 2
worldWidth = 0
worldHeight = 0
worldMap = [[]]
waypoints = [[]]
trajPoints = []
treeNodes = []
initDir = [] # Will be a numpy array
initPos = [] # Will be a numpy array


class DrawingWin(Gtk.Window):
    def __init__(self):
        super(DrawingWin, self).__init__()
        self.initUi()
        
        
    def initUi(self):
        darea = Gtk.DrawingArea()
        darea.connect("draw", self.onDraw)
        self.add(darea)

        self.set_title("DIRT Visualizer")
        self.resize(worldWidth * SCALE, worldHeight * SCALE)
        self.set_position(Gtk.WindowPosition.CENTER)
        self.connect("destroy", Gtk.main_quit)
        self.show_all()
        
    
    def onDraw(self, wid, cr):
        # Flip the Context about the x-axis so that right and up are positive directions
        cr.translate(0, worldHeight * SCALE)
        cr.transform(REFLECT_X)

        # Color background
        setDrawColor(cr, "background")
        cr.paint()

        # Calculate cell size for drawing obstacles
        cellSize = worldWidth // len(worldMap[0])

        # Draw obstacles
        for row in range(len(worldMap)):
            for col in range(len(worldMap[0])):
                if (worldMap[row][col] == '#'):
                    x = col * cellSize
                    y = worldHeight - (row * cellSize) - cellSize
                    cr.rectangle(x * SCALE, y * SCALE, cellSize * SCALE, cellSize * SCALE)
        
        # Color obstacles
        setDrawColor(cr, "obstacle")
        cr.fill()

        # Set up for drawing text
        cr.select_font_face("Monospace", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_NORMAL)
        cr.set_font_size(20)
        setDrawColor(cr, "waypoint")
        
        # Draw waypoints
        for i in range(len(waypoints)):
            pt = waypoints[i]
            x = pt[0] * SCALE
            y = pt[1] * SCALE
            r = pt[2] * SCALE
            cr.move_to(x + r, y) # Arcs start drawing from the point at 0 rad, so shift over by radius
            cr.arc(x, y, r, 0, 2 * pi) # Arc from 0 to 2pi radians is a circle
            cr.stroke()
            setDrawColor(cr, "text")
            drawText(cr, x - 6, y - 7, str(i + 1)) # Draw waypoint number in center

        # Draw tree nodes
        cr.set_source_rgb(1, 0, 1)
        cr.set_line_width(0.5)
        for node in treeNodes:
            x = node[0] * SCALE
            y = node[1] * SCALE
            r = node[2] * SCALE
            cr.move_to(x + r, y)
            cr.arc(x, y, r, 0, 2 * pi)
            cr.stroke()
            #cr.fill()

        # Move to the starting point of the vehicle to start drawing path
        cr.move_to(initPos[0] * SCALE, initPos[1] * SCALE)
        cr.set_line_width(1)

        # Draw solution trajectory
        for pt in trajPoints:
            x = pt[0] * SCALE
            y = pt[1] * SCALE
            cr.line_to(x, y)

        setDrawColor(cr, "path")
        cr.stroke()

        # Draw solution node circles over in green
        for pt in trajPoints:
            x = pt[0] * SCALE
            y = pt[1] * SCALE
            r = pt[2] * SCALE
            cr.set_line_width(1.5)
            cr.move_to(x + r, y)
            cr.arc(x, y, r, 0, 2 * pi)

        cr.set_source_rgb(0, 1, 0)
        cr.stroke()

        # Draw vehicle
        x = initPos[0]
        y = initPos[1]
        xDir = initDir[0]
        yDir = initDir[1]
        theta = atan2(yDir, xDir)
        setDrawColor(cr, "vehicle")
        drawTriangle(cr, x * SCALE, y * SCALE, theta)
        
        
def drawText(cr, x, y, text):
    cr.move_to(x, y) # Go to the position for drawing
    cr.save()
    cr.translate(x, y) # Move Context to the position for reflection
    cr.transform(REFLECT_X) # Reflect Context about the x-axis (so the text does not draw upside-down)
    cr.show_text(text) # Draw the text
    cr.restore() # Undo translate and transform of Context


def drawTriangle(cr, x, y, theta):
    cr.save()
    cr.translate(x, y)
    cr.rotate(-pi / 2 + theta)
    cr.move_to(-(TRI_SIZE / 2), -(TRI_SIZE / 2))
    cr.line_to(TRI_SIZE / 2, -(TRI_SIZE / 2))
    cr.line_to(0, TRI_SIZE / 1.5)
    cr.close_path()
    cr.stroke()
    cr.restore()


def setDrawColor(cr, forWhat):
    colors = COLOR_MODES[colorMode]
    
    # Depending on the color mode and what is going to be drawn, apply the correct color
    if (colors == "grayscale"):
        if (forWhat == "background"):
            cr.set_source_rgb(0.95, 0.95, 0.95) # Light gray
        elif (forWhat == "waypoint"):
            cr.set_source_rgb(0.6, 0.6, 0.6) # Gray
        elif (forWhat == "path"):
            cr.set_source_rgb(0.8, 0.8, 0.8) # Light gray
        elif (forWhat == "obstacle"):
            cr.set_source_rgb(0.4, 0.4, 0.4) # Dark gray
        else: # Text, vehicle, waypointHit
            cr.set_source_rgb(0, 0, 0) # Black
    elif (colors == "neon"):
        if (forWhat == "background"):
            cr.set_source_rgb(0, 0, 0) # Black
        elif (forWhat == "waypoint"):
            cr.set_source_rgb(0, 1, 1) # Cyan
        elif (forWhat == "waypointHit"):
            cr.set_source_rgb(1, 1, 0) # Yellow
        elif (forWhat == "path"):
            cr.set_source_rgb(1, 0.4, 0) # Orange
        elif (forWhat == "text"):
            cr.set_source_rgb(1, 0, 1) # Magenta
        elif (forWhat == "obstacle"):
            cr.set_source_rgb(0, 0, 1) # Blue
        else: # Vehicle
            cr.set_source_rgb(0, 1, 0) # Green
    else: # UNH
        if (forWhat == "background"):
            cr.set_source_rgb(1, 1, 1) # White
        elif (forWhat == "waypoint"):
            cr.set_source_rgb(0.6, 0.6, 0.6) # Gray
        elif (forWhat == "waypointHit"):
            cr.set_source_rgb(0, 1, 1) # Cyan
        elif (forWhat == "path"):
            cr.set_source_rgb(0.49, 0.75, 0.93) # Sky blue
        elif (forWhat == "text"):
            cr.set_source_rgb(0.6, 0.6, 0.6) # Gray
        elif (forWhat == "obstacle"):
            cr.set_source_rgb(0, 0, 1) # Blue
        else: # Vehicle
            cr.set_source_rgb(0, 0.75, 1) # Deep sky blue


def readStandardIn():
    global worldWidth, worldHeight, worldMap, waypoints, initDir, initPos, trajPoints, treeNodes
    fillMode = "none"
    firstLine = True
    mapRow = 0
    i = 0
    
    if (len(sys.argv) > 2):
        # Read from first command line argument (problem file)
        with open(sys.argv[1], "r") as file:
            for line in file:
                # Strip newline from line and split by whitespace
                line = line.strip().split()
                
                # Read in data
                if (line != []):
                    if (fillMode == "none"):
                        if (line[0] == "WORLD_DIMENSIONS:"):
                            worldWidth = int(line[1])
                            worldHeight = int(line[2])
                        elif (line[0] == "MAP"):
                            fillMode = "worldMap"
                        elif (line[0] == "INITIAL_DIR:"):
                            theta = radians(float(line[1]))
                            x = cos(theta)
                            y = sin(theta)
                            initDir = np.array([x, y])
                        elif (line[0] == "INITIAL_POS:"):
                            initPos = np.array([float(line[1]), float(line[2])])
                        elif (line[0] == "NUM_WAYPOINTS:"):
                            waypoints = [None for j in range(int(line[1]))]
                        elif (line[0] == "WAYPOINTS"):
                            fillMode = "waypoints"
                    elif (fillMode == "worldMap"):
                        # Initialize worldMap to the correct dimensions
                        if (worldMap == [[]]):
                            mapWidth = len(line[0])
                            scaleFactor = worldWidth // mapWidth
                            mapHeight = worldHeight // scaleFactor
                            worldMap = [None for j in range(mapHeight)]

                        # Convert the line into an array of characters to be the map row
                        worldMap[mapRow] = list(line[0])
                        mapRow += 1

                        # Stop filling worldMap when it's full
                        if (mapRow >= len(worldMap)):
                            fillMode = "none"
                    else: # Fill waypoints
                        line.pop(0) # Get rid of first element of line (waypoint number)
                        waypoints[i] = [float(strNum) for strNum in line]
                        i += 1
                        
                        # Stop filling waypoints when it's full
                        if (i >= len(waypoints)):
                            fillMode = "none"

        # Read from second command line argument (tree node file)
        with open(sys.argv[2], "r") as file:
            for line in file:
                # Strip newline from line and split by whitespace
                line = line.strip().split()

                # Read in data
                if (line != []):
                    if (line[0] == "TREE_NODES"):
                        fillMode = "treeNodes"
                    elif (line[0] == "TRAJECTORY"):
                        fillMode = "trajPoints"
                    elif (fillMode == "treeNodes"):
                        treeNodes.append([float(strNum) for strNum in line])
                    elif (fillMode == "trajPoints"):
                        trajPoints.append([float(strNum) for strNum in line])
    else:
        # If too few command-line arguments are given, inform the user and exit the program with error status
        print("Missing command-line arguments. The first should be the problem file, and the second should be the tree node file.")
        sys.exit(1)


def main():
    readStandardIn()
    win = DrawingWin()
    Gtk.main()
    
        
if __name__ == "__main__":    
    main()