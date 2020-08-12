import cairo
from math import pi
import fileinput

## Program that reads in a file containing vehicle states from command line and creates a simple pdf visualization.
## Author: Bryan McKenney
    
def main():
    WORLD_WIDTH = 50
    WORLD_HEIGHT = 50
    TRI_SIZE = 20
    ZOOM = 100
    OFFSET = 0
    pts = [[]]
    i = 0
    
    # Read vehicle states from file into a 2D array
    for line in fileinput.input():
        line = line.strip().split()
        if (len(line) == 1):
            pts = [None for j in range(int(line[0]))]
        else:
            pts[i] = [float(numStr) * ZOOM + OFFSET for numStr in line] # Convert strings into floats (with zoom and offset)
            i += 1
    
    # Create a pdf surface with dimensions equal to the world size
    ps = cairo.PDFSurface("visualization.pdf", WORLD_WIDTH, WORLD_HEIGHT)
    cr = cairo.Context(ps)
    
    # Set (0, 0) to bottom left corner (positive going up and right)
    cr.translate(0, WORLD_HEIGHT)
    mtx = cairo.Matrix(1, 0, 0, -1) # Matrix to reflect over x-axis
    cr.transform(mtx)
    
    # Set draw color and line thickness
    cr.set_source_rgb(0, 0, 0)
    cr.set_line_width(0.5)
    
    # Draw triangles
    for pt in pts:
        cr.save()
        cr.translate(pt[0], pt[1])
        cr.rotate(-pi/2 + (pt[2] - OFFSET) / ZOOM)
        cr.move_to(0 - (TRI_SIZE / 2), 0 - (TRI_SIZE / 2))
        cr.line_to(0 + (TRI_SIZE / 2), 0 - (TRI_SIZE / 2))
        cr.line_to(0, 0 + (TRI_SIZE / 1.5))
        cr.close_path()
        cr.restore()
        cr.stroke()
    
    cr.show_page()
    
if __name__ == "__main__":
    main()
