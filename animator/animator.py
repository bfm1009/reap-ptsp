import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk, Gdk, GLib
import cairo
from PIL import Image
from math import *
import sys
import os
import numpy as np
from vehicle import Vehicle

## Program that reads in two files -- one containing waypoints and one containing vehicle instructions -- and animates the motion of the vehicle.
## Author: Bryan McKenney

# Global constants
SCALE = 10 # Scale the world size to the screen by this amount
REFLECT_X = cairo.Matrix(1, 0, 0, -1) # Matrix to reflect something over the x-axis
TRI_SIZE = 20 # The size to draw the triangle representing the vehicle
COLOR_MODES = ["grayscale", "neon", "unh"] # The different color modes that the animation can be displayed in

# Global variables
colorMode = 2
showAnim = True # Whether or not the animation window will open
createGif = True # Whether or not a gif will be generated
worldWidth = 0
worldHeight = 0
worldMap = [[]]
waypoints = [[]]
controls = [[]]
controlNum = 0
initDir = [] # Will be a numpy array
initPos = [] # Will be a numpy array
vehicle = None
currControlTime = 0
paused = True
pathPoints = [] # All the points that the vehicle has visted on its voyage, in order (will be 2D array)
waypointsHit = [] # All the waypoints that the vehicle has visted on its voyage, in order (based on what it aims for, not what it touches)
frameNum = 0


class DrawingWin(Gtk.Window):
    def __init__(self):
        super(DrawingWin, self).__init__()
        self.initUi()
        
        
    def initUi(self):
        global paused

        darea = Gtk.DrawingArea()
        darea.connect("draw", self.onDraw)
        darea.set_events(Gdk.EventMask.BUTTON_PRESS_MASK) # Enable button press events
        darea.connect("button-press-event", self.onButtonPress)
        GLib.timeout_add(50, self.tick) # Call tick every 50 milliseconds
        self.add(darea)

        self.set_title("PTSP Animator")
        self.resize(worldWidth * SCALE, worldHeight * SCALE)
        self.set_position(Gtk.WindowPosition.CENTER)
        self.connect("destroy", Gtk.main_quit)
        self.show_all()
        

    def tick(self):
        ## Invalidates the screen, causing the "draw" event to fire
        rect = self.get_allocation()
        
        if (self.get_window() != None): # Stops the program from crashing when closed
            self.get_window().invalidate_rect(rect, True)
            return True # Causes timeout to tick again
        else:
            return False


    def onButtonPress(self, w, e):
        global vehicle, controls, controlNum, currControlTime, initDir, initPos, paused, colorMode, pathPoints, waypointsHit, frameNum, createGif

        # Codes for left and right mouse buttons
        MB_LEFT = 1
        MB_RIGHT = 3

        if (e.type == Gdk.EventType.BUTTON_PRESS):
            # On a left click, control animation
            if (e.button == MB_LEFT):
                # If the animation is finished, reset it; otherwise, toggle paused
                if (controlNum == len(controls) and currControlTime <= 0):
                    vehicle = Vehicle(initDir, np.array([0, 0]), initPos)
                    controlNum = 0
                    currControlTime = 0
                    pathPoints = []
                    waypointsHit = []
                    createGif = False # Only create a GIF the first time the animation is run
                else:
                    paused = not paused
                    
            # On a right click, cycle through color modes
            elif (e.button == MB_RIGHT):
                if (colorMode == len(COLOR_MODES) - 1):
                    colorMode = 0
                else:
                    colorMode += 1
        
    
    def onDraw(self, wid, cr):
        global vehicle, controls, controlNum, currControlTime, paused, frameNum

        # Create another Context for drawing to a PNG as well as the screen
        if (createGif):
            ims = cairo.ImageSurface(cairo.FORMAT_RGB24, worldWidth * SCALE, worldHeight * SCALE)
            crPNG = cairo.Context(ims)
        
        # Flip the Context about the x-axis so that right and up are positive directions
        cr.translate(0, worldHeight * SCALE)
        cr.transform(REFLECT_X)

        if (createGif):
            crPNG.translate(0, worldHeight * SCALE)
            crPNG.transform(REFLECT_X)

        # Update the vehicle if the animation is unpaused
        if (not paused):
            # Subtract how much time has passed since the last update
            currControlTime -= 0.05

            # Update vehicle controls if the last controls have been held for the specified time
            if (currControlTime <= 0):
                if (controlNum < len(controls)):
                    turn = controls[controlNum][0]
                    acceleration = controls[controlNum][1]
                    currControlTime = controls[controlNum][2]

                    vehicle.setControls(turn, acceleration)
                    controlNum += 1
                else:
                    # Freeze at the last frame of the animation
                    paused = True

                    if (createGif):
                        # Create list to store GIF frames in
                        frames = []

                        # Fill frames with the saved PNGs and delete them
                        for i in range(frameNum):
                            filename = f"./frames/frame{i}.png"
                            frame = Image.open(filename)

                            # If the first or last frame, add it 10 times to freeze there for 0.5 seconds
                            if (i == 0 or i == frameNum - 1):
                                for j in range(10):
                                    frames.append(frame)
                            else:
                                frames.append(frame)

                            os.remove(filename)

                        # Save the frames as an animating GIF (at 20 fps, so realtime)
                        frames[0].save("animation.gif",
                                        save_all = True,
                                        append_images = frames[1:],
                                        duration = 50,
                                        loop = 0)
                    
            # Update state of vehicle and add its new position to pathPoints
            if (not paused):
                vehicle.updateState()
                pathPoints.append([vehicle.pos[0], vehicle.pos[1]])

                # If the last state ended in a waypoint, record that that waypoint has been hit
                if (controlNum > 0 and currControlTime <= 0.05):
                    waypointHit = controls[controlNum - 1][3]
                    if (waypointHit != 0):
                        waypointsHit.append(waypointHit)

        # Color background
        setDrawColor(cr, "background")
        cr.paint()

        if (createGif):
            setDrawColor(crPNG, "background")
            crPNG.paint()

        # Calculate cell size for drawing obstacles
        cellSize = worldWidth // len(worldMap[0])

        # Draw obstacles
        for row in range(len(worldMap)):
            for col in range(len(worldMap[0])):
                if (worldMap[row][col] == '#'):
                    x = col * cellSize
                    y = worldHeight - (row * cellSize) - cellSize
                    cr.rectangle(x * SCALE, y * SCALE, cellSize * SCALE, cellSize * SCALE)

                    if (createGif):
                        crPNG.rectangle(x * SCALE, y * SCALE, cellSize * SCALE, cellSize * SCALE)
        
        # Color obstacles
        setDrawColor(cr, "obstacle")
        cr.fill()

        if (createGif):
            setDrawColor(crPNG, "obstacle")
            crPNG.fill()

        # Set up for drawing text
        cr.select_font_face("Monospace", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_NORMAL)
        cr.set_font_size(20)

        if (createGif):
            crPNG.select_font_face("Monospace", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_NORMAL)
            crPNG.set_font_size(20)
        
        # Draw waypoints
        for i in range(len(waypoints)):
            pt = waypoints[i]
            x = pt[0] * SCALE
            y = pt[1] * SCALE
            r = pt[2] * SCALE
            cr.move_to(x + r, y) # Arcs start drawing from the point at 0 rad, so shift over by radius
            cr.arc(x, y, r, 0, 2 * pi) # Arc from 0 to 2pi radians is a circle

            try: # Determine if this waypoint has been hit by the vehicle or not
                waypointsHit.index(i + 1)
                hit = True
            except ValueError:
                hit = False
            
            if (hit): # Change the color of the waypoint if it has been hit
                setDrawColor(cr, "waypointHit")
            else:
                setDrawColor(cr, "waypoint")

            cr.stroke()
            setDrawColor(cr, "text")
            drawText(cr, x - 6, y - 7, str(i + 1)) # Draw waypoint number in center

            if (createGif):
                crPNG.move_to(x + r, y)
                crPNG.arc(x, y, r, 0, 2 * pi)
                
                if (hit):
                    setDrawColor(crPNG, "waypointHit")
                else:
                    setDrawColor(crPNG, "waypoint")

                crPNG.stroke()
                setDrawColor(crPNG, "text")
                drawText(crPNG, x - 6, y - 7, str(i + 1))

        # Move to the starting point of the vehicle to start drawing path
        cr.move_to(initPos[0] * SCALE, initPos[1] * SCALE)

        if (createGif):
            crPNG.move_to(initPos[0] * SCALE, initPos[1] * SCALE)

        # Draw vehicle path
        for pt in pathPoints:
            x = pt[0]
            y = pt[1]
            cr.line_to(x * SCALE, y * SCALE)

            if (createGif):
                crPNG.line_to(x * SCALE, y * SCALE)

        # Color path
        setDrawColor(cr, "path")
        cr.stroke()

        if (createGif):
            setDrawColor(crPNG, "path")
            crPNG.stroke()

        # Draw vehicle
        x = vehicle.pos[0]
        y = vehicle.pos[1]
        xDir = vehicle.dir[0]
        yDir = vehicle.dir[1]
        theta = atan2(yDir, xDir)
        setDrawColor(cr, "vehicle")
        drawTriangle(cr, x * SCALE, y * SCALE, theta)

        if (createGif):
            setDrawColor(crPNG, "vehicle")
            drawTriangle(crPNG, x * SCALE, y * SCALE, theta)

        # Draw vehicle location on screen but not in GIF
        cr.set_font_size(12)
        setDrawColor(cr, "text")
        drawText(cr, 10, worldHeight * SCALE - 20, "(%.2f, %.2f) Rad: %.2f" % (x, y, theta))

        # Save frame as a PNG
        if (createGif and (not paused or frameNum == 0)):
            ims.write_to_png(f"./frames/frame{frameNum}.png")
            frameNum += 1
        
        
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
    global worldWidth, worldHeight, worldMap, waypoints, controls, initDir, initPos, vehicle, showAnim
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

        # Read from second command line argument (control file)
        with open(sys.argv[2], "r") as file:
            for line in file:
                # Strip newline from line and split by whitespace
                line = line.strip().split()

                # Read in data
                if (firstLine):
                    controls = [None for j in range(int(line[0]))]
                    firstLine = False
                    i = 0 # Reset i
                else:
                    controls[i] = [float(strNum) for strNum in line]
                    i += 1

        # Initialize the vehicle
        vehicle = Vehicle(initDir, np.array([0, 0]), initPos)

        # Option to not display the animation (will only create gif)
        if (len(sys.argv) > 3 and sys.argv[3] == "false"):
           showAnim = False
    else:
        # If too few command-line arguments are given, inform the user and exit the program with error status
        print("Missing command-line arguments. The first should be the problem file, and the second should be the control file.")
        sys.exit(1)


def justCreateGif():
    global vehicle, controls, controlNum, currControlTime, frameNum

    while (True):
        # Create a Context for drawing to a PNG
        ims = cairo.ImageSurface(cairo.FORMAT_RGB24, worldWidth * SCALE, worldHeight * SCALE)
        crPNG = cairo.Context(ims)
        
        # Flip the Context about the x-axis so that right and up are positive directions
        crPNG.translate(0, worldHeight * SCALE)
        crPNG.transform(REFLECT_X)

        # Subtract how much time has passed since the last update
        currControlTime -= 0.05

        # Update vehicle controls if the last controls have been held for the specified time
        if (currControlTime <= 0):
            if (controlNum < len(controls)):
                turn = controls[controlNum][0]
                acceleration = controls[controlNum][1]
                currControlTime = controls[controlNum][2]

                vehicle.setControls(turn, acceleration)
                controlNum += 1
            else:
                # Create list to store GIF frames in
                frames = []

                # Fill frames with the saved PNGs and delete them
                for i in range(frameNum):
                    filename = f"./frames/frame{i}.png"
                    frame = Image.open(filename)

                    # If the first or last frame, add it 10 times to freeze there for 0.5 seconds
                    if (i == 0 or i == frameNum - 1):
                        for j in range(10):
                            frames.append(frame)
                    else:
                        frames.append(frame)

                    os.remove(filename)

                # Save the frames as an animating GIF (at 20 fps, so realtime)
                frames[0].save("animation.gif",
                                save_all = True,
                                append_images = frames[1:],
                                duration = 50,
                                loop = 0)

                # We're done here
                print("Gif created\n")
                break
                
        # Update state of vehicle and add its new position to pathPoints
        vehicle.updateState()
        pathPoints.append([vehicle.pos[0], vehicle.pos[1]])

        # If the last state ended in a waypoint, record that that waypoint has been hit
        if (controlNum > 0 and currControlTime <= 0.05):
            waypointHit = controls[controlNum - 1][3]
            if (waypointHit != 0):
                waypointsHit.append(waypointHit)

        # Color background
        setDrawColor(crPNG, "background")
        crPNG.paint()

        # Calculate cell size for drawing obstacles
        cellSize = worldWidth // len(worldMap[0])

        # Draw obstacles
        for row in range(len(worldMap)):
            for col in range(len(worldMap[0])):
                if (worldMap[row][col] == '#'):
                    x = col * cellSize
                    y = worldHeight - (row * cellSize) - cellSize
                    crPNG.rectangle(x * SCALE, y * SCALE, cellSize * SCALE, cellSize * SCALE)

        # Color obstacles
        setDrawColor(crPNG, "obstacle")
        crPNG.fill()

        # Set up for drawing text
        crPNG.select_font_face("Monospace", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_NORMAL)
        crPNG.set_font_size(20)

        # Draw waypoints
        for i in range(len(waypoints)):
            pt = waypoints[i]
            x = pt[0] * SCALE
            y = pt[1] * SCALE
            r = pt[2] * SCALE
            crPNG.move_to(x + r, y) # Arcs start drawing from the point at 0 rad, so shift over by radius
            crPNG.arc(x, y, r, 0, 2 * pi) # Arc from 0 to 2pi radians is a circle

            try: # Determine if this waypoint has been hit by the vehicle or not
                waypointsHit.index(i + 1)
                hit = True
            except ValueError:
                hit = False
            
            if (hit): # Change the color of the waypoint if it has been hit
                setDrawColor(crPNG, "waypointHit")
            else:
                setDrawColor(crPNG, "waypoint")

            crPNG.stroke()
            setDrawColor(crPNG, "text")
            drawText(crPNG, x - 6, y - 7, str(i + 1))

        # Move to the starting point of the vehicle to start drawing path
        crPNG.move_to(initPos[0] * SCALE, initPos[1] * SCALE)

        # Draw vehicle path
        for pt in pathPoints:
            x = pt[0]
            y = pt[1]
            crPNG.line_to(x * SCALE, y * SCALE)

        # Color path
        setDrawColor(crPNG, "path")
        crPNG.stroke()

        # Draw vehicle
        x = vehicle.pos[0]
        y = vehicle.pos[1]
        xDir = vehicle.dir[0]
        yDir = vehicle.dir[1]
        theta = atan2(yDir, xDir)
        setDrawColor(crPNG, "vehicle")
        drawTriangle(crPNG, x * SCALE, y * SCALE, theta)

        # Save frame as a PNG
        ims.write_to_png(f"./frames/frame{frameNum}.png")
        frameNum += 1


def main():
    readStandardIn()

    # Either open interactive animation window and create gif or just create gif
    if (showAnim):
        win = DrawingWin()
        Gtk.main()
    else:
        print("Creating gif...")
        justCreateGif()
    
        
if __name__ == "__main__":    
    main()