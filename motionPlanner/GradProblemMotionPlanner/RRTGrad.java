import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Random;
import java.util.Scanner;

/**
 * This program uses the RRT (rapidly exploring random trees) algorithm to create a motion tree of a valid
 * solution trajectory that is bound by certain constraints. For right now, the vehicle has a limited acceleration and
 * an essentially unlimited turning radius. It navigates around obstacles (represented by "#" in the map), and considers
 * the goal to be reached when it gets within a distance of 0.1 units.
 *
 * @author Lucas Guerrette
 * @version GradProblem !!!!(not PTSP vehicle yet)!!!!
 *
 */
public class RRTGrad {
    //Fields
    ArrayList<Vertex> points = new ArrayList<>(); //ArrayList containing all of the vertices on the graph
    ArrayList<double[]> edges = new ArrayList<>(); //ArrayList containing all of the edges on the graph
    ArrayList<int[]> blockedSpaces; // ArrayList containing the coordinates of all blocked spaces
    Vertex root; //Root of the Tree
    Vertex goal; // Goal node that is trying to be reached
    String map = ""; // The map of the world, which is read in from a text file
    int width; //Map width
    int height; //Map height
    double stepSlice = 0.05; // The "time" between each state update inside of getBestControl()


    /**
     * This method reads in the map from the given text file. The map string gets created in the main method, and is
     * fed into here to find the blocked spaces.
     * @param map A string containing the map of the world
     * @return An ArrayList of integer arrays, with each array containing the x and y coordinates of the bottom left
     * point of each obstacle
     */
    public ArrayList<int[]> readMap(String map) {
        blockedSpaces = new ArrayList<>();
        int y = 1;
        for (int h = height - 1; h >= 0; h--) {
            for (int w = h * width; w < (h * width) + width; w++) {
                if (map.charAt(w) == '#') {
                    blockedSpaces.add(new int[] {w % width + 1, y});
                }
            }
            y++;
        }
        return blockedSpaces;
    }

    /**
     * This method simply adds a new edge to the tree, containing the vertex the controls are being applied to, and the
     * controls themselves.
     * @param v The vertex that the moves are applied to
     * @param controls the set of (Angle, Magnitude) controls that were applied to the given vertex
     */
    public void addEdge(Vertex v, double[] controls) {
        edges.add(new double[] {v.xPos, v.yPos, v.xVel, v.yVel, controls[4], controls[5]});
    }

    /**
     * This method takes a vertex and checks to see if it has collided with an obstacle, or if it is out of bounds.
     * @param v a Vertex
     * @return true if there is a collision, false otherwise
     */
    public boolean vertexCollision(Vertex v) {
        if (v.xPos < 0 || v.xPos > width || v.yPos < 0 || v.yPos > height) {
            return true;
        }
        for (int i = 0; i < blockedSpaces.size(); i++) {
            if (v.xPos < blockedSpaces.get(i)[0] && v.xPos > blockedSpaces.get(i)[0] - 1) {
                if (v.yPos < blockedSpaces.get(i)[1] && v.yPos > blockedSpaces.get(i)[1] - 1) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Takes two doubles, representing the coordinates of a vertex, and test to see if it is out of bounds. This is
     * mainly used during calculation of the trajectory, before an end point is created.
     * @param x the current x position being tested
     * @param y the current y position being tested
     * @return true if there is a collision, false otherwise
     */
    public boolean posCollision(double x, double y) {
        if (x < 0 || x > width || y < 0 || y > height) {
            return true;
        }
        for (int i = 0; i < blockedSpaces.size(); i++) {
            if (x < blockedSpaces.get(i)[0] && x > blockedSpaces.get(i)[0] - 1) {
                if (y < blockedSpaces.get(i)[1] && y > blockedSpaces.get(i)[1] - 1) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * This method just returns a random state on the map that the RRT can aim for, with a 5% chance of that point being
     * the goal.
     * @return A random vertex on a valid position on the map
     */
    public Vertex randomState() {
        Random rand = new Random();
        double randX = rand.nextDouble() * width;
        double randY = rand.nextDouble() * height;
        double vertChance = rand.nextInt(100);
        if (vertChance <= 5) {
            return goal;
        }
        Vertex vRand = new Vertex(randX, randY, 0, 0, 0, 0);
        if (vertexCollision(vRand)) {
            return randomState();
        } else {
            return vRand;
        }
    }

    /**
     * This method goes through the ArrayList of points currently on the graph, and does a simple distance calculation
     * to find which point is closest to the randomly created that is being aimed at.
     * @param xRand The random point that the motion planner is going towards
     * @return Returns the closest vertex to the random point
     */
    public Vertex nearestNeighbor(Vertex xRand) {
        double bestDist = Math.sqrt(Math.pow(width, 2) + Math.pow(height, 2));
        int index = 0;
        for (int i = 0; i < points.size(); i++) {
            double dist = Math.sqrt(Math.pow(xRand.xPos - points.get(i).xPos, 2)
                    + Math.pow(xRand.yPos - points.get(i).yPos, 2));
            if (dist < bestDist) {
                bestDist = dist;
                index = i;
            }
        }
        return points.get(index);
    }

    /**
     * Returns a random angle between 0 and 2PI radians to be used in one of the 10 control sets that gets fed to
     * getBestControl().
     * @return A random angle
     */
    public double randomAngle() {
        Random rand = new Random();
        double x = rand.nextInt(360);
        return Math.toRadians(x);
    }

    /**
     * Similar to randomAngle(), this method returns a random magnitude value between 0 and 0.5 to be put in the control
     * set that gets fed to getBestControl().
     * @return A random magnitude
     */
    public double randomMag() {
        Random rand = new Random();
        return rand.nextDouble() / 2;
    }

    /**
     * This method creates 10 double arrays that each contain a random angle and magnitude to be tested by the
     * getBestControl() method.
     * @return A 2D double array containing 10 pairs of (Angle, Magnitude) controls
     */
    public double[][] getControlSet() {
        double[][] controls = new double[10][4];
        for (int i = 0; i < controls.length - 1; i++) {
            controls[i] = new double[] {randomAngle(), randomMag()};
        }
        return controls;
    }

    /**
     * This method is where a lot of the magic happens. It goes through each of the 10 control sets provided to it,
     * calculates where the controls will end up (doing collision checking along the way), and returns the control that
     * gets the vehicle closest to the randomly chosen point. If all 10 controls end in a collision with something,
     * then nothing is added to the motion tree, and a new random point is picked. Along with the controls, the double
     * array it returns also contains the x and y positions and velocities at the new point that the control reached.
     * @param xNear The closest waypoint in the graph to the randomly selected point
     * @param xRand The randomly selected point
     * @param controlSet The 10 control sets that are being tested
     * @return The single control set that got the vehicle closest to the randomly selected point
     */
    public double[] getBestControl(Vertex xNear, Vertex xRand, double[][] controlSet) {
        double bestDist = width * height;
        int collisionCount = 0;
        double[] stateControl = new double[6];
        for (int i = 0; i < controlSet.length; i++) {
            double xV = xNear.xVel;
            double yV = xNear.yVel;
            double x = xNear.xPos;
            double y = xNear.yPos;
            for (int s = 1; s <= 20; s++) {
                double thetaTotal = Math.atan2(yV, xV) + controlSet[i][0];
                xV = xV + (stepSlice * controlSet[i][1] * Math.cos(thetaTotal));
                yV = yV + (stepSlice * controlSet[i][1] * Math.sin(thetaTotal));
                x = x + (stepSlice * xV);
                y = y + (stepSlice * yV);
                if (posCollision(x, y)) {
                    collisionCount++;
                    break;
                }
            }
            if (!posCollision(x, y)) {
                double dist = Math.sqrt(Math.pow(xRand.xPos - x, 2) + Math.pow(xRand.yPos - y, 2));
                if (dist < bestDist) {
                    bestDist = dist;
                    stateControl = new double[] {x, y, xV, yV, controlSet[i][0], controlSet[i][1]};
                }
            }
        }
        if (collisionCount != controlSet.length) {
            return stateControl;
        } else {
            return null;
        }
    }

    /**
     * Creates a valid new state to be added to the graph based on the double array returned by getBestControl(). It
     * also takes the vertex that the new state came from, and assigns that as the parent of this new node. This is so
     * that the output for the visualizer that was used for the assignment could be formatted correctly, among other
     * things.
     * @param xNear The vertex that this new one came from
     * @param control the double array that came from getBestControl(), containing the new x/y positions/velocities and
     *                the controls that were applied to get it there.
     * @return a new vertex to be added to the graph.
     */
    public Vertex newState(Vertex xNear, double[] control) {
        Vertex xNew = new Vertex(control[0], control[1], control[2], control[3], control[4], control[5]);
        xNew.parent = xNear;
        return xNew;
    }

    /**
     * This is the actual RRT algorithm. For each cycle in the algorithm, a new random point on the map is selected,
     * it's nearest neighbor is selected from points already on the graph, the control that gets the vehicle from the
     * nearest vertex to near the random point is selected, a new node is created from that control, and that node is
     * added to the graph. In this RRT, the algorithm ends either when a vertex reaches the goal, or if no solution is
     * found after a certain number of cycles (usually a few thousand).
     * @param cycles The maximum number of times to run the algorithm
     */
    public void generateRRT(int cycles) {
        for (int i = 0; i <= cycles; i++) {
            Vertex xRand = randomState();
            Vertex xNear = nearestNeighbor(xRand);
            double[] u = getBestControl(xNear, xRand, getControlSet());
            if (u != null && u[4] != 0.0 && u[5] != 0.0) {
                Vertex xNew = newState(xNear, u);
                points.add(xNew);
                addEdge(xNear, u);
                if (reachedGoal(xNew)) {
                    ArrayList<Vertex> solution = new ArrayList<>();
                    while (xNew != null) {
                        solution.add(xNew);
                        xNew = xNew.parent;
                    }
                    System.out.println(solution.size() - 1);
                    for (int k = solution.size() - 2; k >= 0; k--) {
                        System.out.println(solution.get(k).parent.xPos + " " + solution.get(k).parent.yPos
                                + " " + solution.get(k).parent.xVel + " " + solution.get(k).parent.yVel
                                + " " + solution.get(k).angle + " " + solution.get(k).magnitude);
                    }
                    break;
                }
            }
        }
    }

    /**
     * This method tests whether or not a vertex has reached the goal. It is checked against every vertex that is added
     * to the graph.
     * @param v The newest vertex being added to the graph
     * @return True if the vertex has achieved the goal, false otherwise
     */
    public boolean reachedGoal(Vertex v) {
        return Math.sqrt(Math.pow(v.xPos - goal.xPos, 2) + Math.pow(v.yPos - goal.yPos, 2)) <= 0.1;
    }

    /**
     * This method prints out all of the edges that have been added to the graph during the algorithm's run. Mostly
     * used for the visualizer's sake.
     */
    public void printEdges() {
        System.out.println(edges.size());
        for (int k = 0; k < edges.size(); k++) {
            System.out.println(edges.get(k)[0] + " " + edges.get(k)[1] + " " + edges.get(k)[2]
                + " " + edges.get(k)[3] + " " + edges.get(k)[4] + " " + edges.get(k)[5]);
        }
    }

    /**
     * This subclass defines each vertex.
     */
    public static class Vertex {
        double xPos;
        double yPos;
        double xVel;
        double yVel;
        double angle;
        double magnitude;
        Vertex parent;

        public Vertex(double x, double y, double vx, double vy, double ang, double mag) {
            xPos = x;
            yPos = y;
            xVel = vx;
            yVel = vy;
            angle = ang;
            magnitude = mag;
        }
    }

    /**
     * This is the main method that is used when the code runs on Agate. This does not contain any command line output
     * besides the solution trajectory and motion tree.
     */
    public static void mainAgate() {
        Scanner sc = new Scanner(System.in);
        RRTGrad g = new RRTGrad();
        g.width = sc.nextInt();
        g.height = sc.nextInt();
        for (int i = 0; i <= g.height; i++) {
            g.map += sc.nextLine();
        }
        g.readMap(g.map);
        double rX = sc.nextDouble();
        double rY = sc.nextDouble();
        g.root = new Vertex(rX, rY, 0, 0, 0, 0);
        g.root.parent = null;
        g.points.add(g.root);
        double gX = sc.nextDouble();
        double gY = sc.nextDouble();
        g.goal = new Vertex(gX, gY, 0, 0, 0, 0);
        g.goal.parent = null;
        g.generateRRT(5000);
        g.printEdges();
    }

    /**
     * This is the main method for simple testing on my laptop. This just made it easy to read in text files from the
     * IDE.
     */
    public static void mainLaptop() {
        try {
            Scanner sc = new Scanner(System.in);
            System.out.println("Input name of Map Data file:");
            String filename = sc.nextLine();
            File input = new File(filename);
            Scanner fileReader = new Scanner(input);
            RRTGrad g = new RRTGrad();
            g.width = fileReader.nextInt();
            g.height = fileReader.nextInt();
            for (int i = 0; i <= g.height; i++) {
                g.map += fileReader.nextLine();
            }
            g.readMap(g.map);
            double rX = fileReader.nextDouble();
            double rY = fileReader.nextDouble();
            g.root = new Vertex(rX, rY, 0, 0, 0, 0);
            g.root.parent = null;
            g.points.add(g.root);
            double gX = fileReader.nextDouble();
            double gY = fileReader.nextDouble();
            g.goal = new Vertex(gX, gY, 0, 0, 0, 0);
            g.goal.parent = null;
            System.out.println("Map data loaded successfully!");
            System.out.println("Root located at (" + g.root.xPos + ", " + g.root.yPos + ")");
            System.out.println("Goal located at (" + g.goal.xPos + ", " + g.goal.yPos + ")");
            g.generateRRT(5000);
            g.printEdges();

        } catch (FileNotFoundException e) {
            System.out.println("File not found.");
            e.printStackTrace();
        }
    }

    /**
     * The main method, where either mainAgate or mainLaptop are run, depending on where the program is being run.
     * @param args None
     */
    public static void main(String[] args) {
        mainAgate();
        //mainLaptop();
    }
}
