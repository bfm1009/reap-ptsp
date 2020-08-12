package motionPlanner.PTSPVehicleMotionPlanner;

import java.io.FileWriter;
import java.io.IOException;
import java.math.BigDecimal;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.Random;


/**
 * This program uses the RRT (rapidly exploring random trees) algorithm to create a motion tree of a valid
 * solution trajectory that is bound by certain constraints. As of right now, the vehicle has a limited acceleration and
 * turning radius, and considers the goal to have been reached by getting withing 3 units of it. The acceleration and
 * turning values are currently generated randomly. It will probably work on maps with obstacles, but that hasn't been
 * tested yet because the maps would be much larger in unit size that those from the grad student problems (probably
 * hundreds of units wide versus maybe 5 - 10 for the grad problem).
 *
 *
 * @author Lucas Guerrette
 * @version Version 0.5? Upgraded PTSP Vehicle
 *
 */
public class RRT {
    //Fields
    ArrayList<Vertex> points = new ArrayList<>(100000); //ArrayList containing all of the vertices on the graph
    ArrayList<double[]> edges = new ArrayList<>(); //ArrayList containing all of the edges on the graph
    ArrayList<int[]> blockedSpaces; // ArrayList containing the coordinates of all blocked spaces
    Vertex root; //Root of the Tree
    Vertex goal; // Goal node that is trying to be reached
    Vertex last; // Last vertex in solution
    String map; // The map of the world, which is read in from a text file
    int width; //Map width
    int height; //Map height
    int objFactor = 10;
    double stepSlice = 0.05; // The "time" between each state update inside of getBestControl()
    Random r;

    /**
     * The Constructor for the RRT, which takes in two double arrays for the root and goal, the map width, and map
     * height, and sets up the RRT so that it can be ran via the Mastermind program.
     * @param root The double array representing the root of the RRT
     * @param goal The double array representing the goal of the RRT
     * @param mapWidth The map width
     * @param mapHeight The map height
     * @param map The String containing the map data for the world
     */
    public RRT(Vertex root, Vertex goal, int mapWidth, int mapHeight, String map, long seed) {
        width = mapWidth;
        height = mapHeight;
        this.map = map;
        readMap(this.map);
        this.root = root;
        points.add(this.root);
        this.root.parent = null;
        this.goal = goal;
        this.goal.parent = null;
        r = new Random(seed);
    }

    /**
     * A blank contructor, mainly used for testing from the main method of this program file.
     */
    public RRT() {
        //Does nothing yay!
    }

    /**
     * This method is meant to load in a Vertex from the double array that is given by the Necromancer.
     * @param input The double array representing Vector values that the Vertex created by this method will take on
     * @return A Vertex with the values given by the double array
     */
    public static Vertex loadPoint(double[] input) {
        Vector direction = new Vector(input[0], input[1]);
        Vector velocity = new Vector(input[2], input[3]);
        Vector position = new Vector(input[4], input[5]);
        return new Vertex(direction, velocity, position);
    }

    /**
     * This method simply returns a random time step for a control to be tried for.
     * @return A timestep ranging between 0.05 and 0.55
     */
    public double randomTimeStep() {
        return r.nextDouble() / 2 + 0.05;
    }

    /**
     * ----------------------- NOT NEEDED FOR VERSION 0 ------------------------
     * This method reads in the map from the given text file. The map string gets created in the main method, and is
     * fed into here to find the blocked spaces.
     * @param map A string containing the map of the world
     * @return An ArrayList of integer arrays, with each array containing the x and y coordinates of the bottom left
     * point of each obstacle
     */
    public ArrayList<int[]> readMap(String map) {
        blockedSpaces = new ArrayList<>();
        int y = 1;
        for (int h = (height / objFactor) - 1; h >= 0; h--) {
            for (int w = h * width / objFactor; w < (h * width / objFactor) + (width / objFactor); w++) {
                if (map.charAt(w) == '#') {
                    blockedSpaces.add(new int[] {w % (width / objFactor) + 1, y});
                }
            }
            y++;
        }
        return blockedSpaces;
    }

    /**
     * ----------------------- NOT NEEDED FOR VERSION 0 ------------------------
     * This method simply adds a new edge to the tree, containing the vertex the controls are being applied to, and the
     * controls themselves.
     * @param v The vertex that the moves are applied to
     * @param controls the set of (Angle, Magnitude) controls that were applied to the given vertex
     */
    public void addEdge(Vertex v, double[] controls) {
        //edges.add(new double[] {v.xPos, v.yPos, v.xVel, v.yVel, controls[4], controls[5]});
    }

    /**
     * This method takes a vertex and checks to see if it has collided with an obstacle, or if it is out of bounds.
     * @param v a Vertex
     * @return true if there is a collision, false otherwise
     */
    public boolean vertexCollision(Vertex v) {
        if (v.position.x < 0 || v.position.x > width || v.position.y < 0 || v.position.y > height) {
            return true;
        }
        for (int i = 0; i < blockedSpaces.size(); i++) {
            if (v.position.x < blockedSpaces.get(i)[0] * objFactor
                    && v.position.x > (blockedSpaces.get(i)[0] * objFactor - objFactor)) {
                if (v.position.y < blockedSpaces.get(i)[1] * objFactor
                        && v.position.y > (blockedSpaces.get(i)[1] * objFactor - objFactor)) {
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
            if (x < blockedSpaces.get(i)[0] * objFactor && x > (blockedSpaces.get(i)[0] * objFactor) - objFactor) {
                if (y < blockedSpaces.get(i)[1] * objFactor && y > (blockedSpaces.get(i)[1] * objFactor) - objFactor) {
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
        double randX = r.nextDouble() * width;
        double randY = r.nextDouble() * height;
        double vertChance = r.nextInt(100);
        if (vertChance <= 6) {
            return goal;
        }
//        double randDirX = (r.nextInt(3) - 1) * r.nextDouble();
////        double randDirY = Math.sqrt(1 - (randDirX * randDirX));
////        int posNeg = (int) Math.round(Math.random());
////        if (posNeg == 1) {
////            randDirY = -randDirY;
////        }
////        Vertex vRand = new Vertex(new Vector(randDirX, randDirY), blankVect(), new Vector(randX, randY));
        Vertex vRand = new Vertex(blankVect(), blankVect(), new Vector(randX, randY));
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
            double dist = calcEuDist(points.get(i), xRand);
            if (dist < bestDist) {
                bestDist = dist;
                index = i;
            }
        }
        return points.get(index);
    }

    /**
     * This method calculates the distance from a given point to the randomly selected one through Euclidean distance.
     * Currently it takes direction and position into account, with direction having 50% more weight than position. This
     * could change over time with more testing however.
     * @param test The vertex being tested against the random position
     * @param xRand The random position that the graph is trying to grow towards
     * @return a double value that represents how far away the point is based on Euclidean distance
     */
    public double calcEuDist(Vertex test, Vertex xRand) {
        return Math.sqrt(Math.pow((xRand.position.x / width) - (test.position.x / width), 2)
                + Math.pow((xRand.position.y / height) - (test.position.y / height), 2));
//                + 1.5 * Math.pow(xRand.direction.x - test.direction.x, 2)
//                + 1.5 * Math.pow(xRand.direction.y - test.direction.y, 2));
    }

    /**
     * Creates and returns a 0, 0 vector for cases like goals, where velocity and direction are irrelevant.
     * @return a "blank" vector
     */
    public Vector blankVect() {
        return new Vector(0, 0);
    }

    /**
     * This method creates 25 random acceleration and turn values to be tried by getBestControl().
     * @return a 2D double array containing the 25 randomly sampled controls
     */
    public double[][] getControlSets() {
        double[][] controls = new double[50][2];
        controls[0] = new double[] {0, 0};
        for (int i = 1; i < controls.length; i++) {
            double acc = r.nextDouble() / 10;
            if (r.nextInt(100) <= 5) {
                acc = 0;
            }
            double turn = (r.nextInt(3) - 1) * r.nextDouble();
            if (r.nextInt(100) <= 5) {
                turn = 0;
            }
            controls[i] = new double[] {acc, turn};
        }
        return controls;
    }

    /**
     * This method is where a lot of the magic happens. It goes through each of the 6 possible control sets provided to
     * it, calculates where the controls will end up (doing collision checking along the way), and returns the control
     * that gets the vehicle closest to the randomly chosen point. If all 6 controls end in a collision with something,
     * then nothing is added to the motion tree, and a new random point is picked. Along with the controls, the double
     * array it returns also contains the x and y positions, velocities and directions at the new point that the control
     * reached.
     * @param xNear The closest vertex in the graph to the randomly selected point
     * @param xRand The randomly selected point
     * @param allControlSets The set of 25 random controls to test
     * @return The single control set that got the vehicle closest to the randomly selected point
     */
    public double[] getBestControl(Vertex xNear, Vertex xRand, double[][] allControlSets) {
        double bestDist = width * height;
        boolean collision = false;
        double[] state = new double[9];
        double rStep = randomTimeStep();
        for (int i = 0; i < allControlSets.length; i++) {
            Vertex copy = xNear.copyVertex();
            for (double s = 0; s < rStep; s += stepSlice) {
                copy.updateDirectionVec(copy.direction, allControlSets[i][1]);
                copy.updateVelocityVec(copy.velocity, copy.direction, allControlSets[i][0]);
                copy.updatePositionVec(copy.position, copy.velocity);
                if (posCollision(copy.position.x, copy.position.y)) {
                    collision = true;
                    break;
                }
            }
            if (!posCollision(copy.position.x, copy.position.y)) {
                double dist = calcEuDist(copy, xRand);
                if (dist < bestDist) {
                    bestDist = dist;
                    state = new double[] {copy.direction.x, copy.direction.y, copy.velocity.x,
                        copy.velocity.y, copy.position.x, copy.position.y, allControlSets[i][0],
                        allControlSets[i][1], rStep};
                }
            }
        }
        if (!collision) {
            return state;
        } else {
            return null;
        }
    }

    /**
     * Drops a Vertex at the halfway mark of the best state returned from getBestState().
     * @param state The best state from getBestState()
     * @param xNear the Vertex that state came from
     * @return a double[] containing the info of a new Vertex
     */
    public double[] dropPoint(double[] state, Vertex xNear) {
        Vertex v = xNear.copyVertex();
        for (double s = 0; s < state[8] / 2; s += stepSlice) {
            v.updateDirectionVec(v.direction, state[7]);
            v.updateVelocityVec(v.velocity, v.direction, state[6]);
            v.updatePositionVec(v.position, v.velocity);
        }
        return new double[] {v.direction.x, v.direction.y, v.velocity.x, v.velocity.y, v.position.x, v.position.y,
                state[6], state[7], state[8] / 2};
    }

    /**
     * Creates a valid new state to be added to the graph based on the double array returned by getBestControl(). It
     * also takes the vertex that the new state came from, and assigns that as the parent of this new node. This is so
     * that the output for the visualizer that was used for the assignment could be formatted correctly, among other
     * things.
     * @param xNear The vertex that this new one came from
     * @param control the double array that came from getBestControl(), containing the new x/y positions/velocities/
     *               directions and the controls that were applied to get it there.
     * @return a new vertex to be added to the graph.
     */
    public Vertex newState(Vertex xNear, double[] control) {
        Vector dir = new Vector(control[0], control[1]);
        Vector vel = new Vector(control[2], control[3]);
        Vector pos = new Vector(control[4], control[5]);
        Vertex xNew = new Vertex(dir, vel, pos);
        xNew.timeStep = control[8];
        xNew.control = new double[] {control[6], control[7]};
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
     * @return An ArrayList of the Vertices that make up the solution path
     */
    public LinkedList<Vertex> generateRRT(int cycles) {
        for (int i = 0; i <= cycles; i++) {
            if (i % 10000 == 0) {
                System.out.println("Iteration: " + i);
            }
            Vertex xRand = randomState(); // Gets the random state to control towards
            Vertex xNear = nearestNeighbor(xRand); // Calculates the state in the tree closest to the random one
            double[] u = getBestControl(xNear, xRand, getControlSets()); // Gets the best controls that steer towards ^
            if (u != null) {
                Vertex xNew = newState(xNear, u); // Makes a new state from the controls
                points.add(xNew);
                if (xNew.timeStep >= 0.1) {
                    points.add(newState(xNear, dropPoint(u, xNear))); // Adds a point at the halfway mark of the control
                }
                //addEdge(xNear, u);
                if (reachedGoalState(xNew)) {
                    LinkedList<Vertex> solution = new LinkedList<>();
                    last = xNew;
                    while (xNew != null) {
                        solution.addFirst(xNew);
                        xNew = xNew.parent;
                    }
                    System.out.println("Solution found during iteration " + i);
                    return solution;
                }
            }
        }
        System.out.println("No solution found within iteration limit.");
        return null;
    }

    /**
     * This method tests whether or not a vertex has reached the goal. It is checked against every vertex that is added
     * to the graph.
     * @param v The newest vertex being added to the graph
     * @return True if the vertex has achieved the goal, false otherwise
     */
    public boolean reachedGoalState(Vertex v) {
        boolean distCheck = Math.sqrt(Math.pow(v.position.x - goal.position.x, 2)
                + Math.pow(v.position.y - goal.position.y, 2)) <= 3;
        //boolean dirXCheck = v.direction.x <= goal.direction.x + 0.15 && v.direction.x >= goal.direction.x - 0.15;
        //boolean dirYCheck = v.direction.y <= goal.direction.y + 0.15 && v.direction.y >= goal.direction.y - 0.15;
        return distCheck; //&& dirXCheck && dirYCheck;
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
     * This subclass defines each vertex. Each Vertex has three vectors, one for direction, velocity, and position.
     * The class also contains the methods that do the calculations on each of those vectors.
     */
    public static class Vertex {
        Vector direction;
        Vector velocity;
        Vector position;
        Vertex parent;
        double timeStep;
        double[] control;
        double friction = 0.99; // The force of friction applied to the velocity values each timestep.
        int waypointHit = 0; // The number of the waypoint hit at this Vertex (0 for none)


        public Vertex(Vector direction, Vector velocity, Vector position) {
            this.direction = direction;
            this.velocity = velocity;
            this.position = position;
        }

        /**
         * This method takes this Vertex's direction vector and an integer (either -1, 0, or 1), and changes the values
         * of the direction vector based on which way it is turning. It is SUPPOSED to be -1 for left, 1 for right, and
         * 0 for no change to direction, which it is, but I had to change the values for the lAlpha and rAlpha to get it
         * that way.
         * @param direction This vertex's direction vector
         * @param turn which direction to turn based on one of the three options.
         */
        public void updateDirectionVec(Vector direction, double turn) { // Vector parameter isn't necessary
            direction.multFour(Math.cos(turn), -Math.sin(turn), Math.sin(turn), Math.cos(turn));
        }

        /**
         * This method takes the direction vector and whether or not the vehicle is accelerating (0 or 1), and updates
         * the velocity vector based on those numbers. It has to make a copy of the direction vector, because otherwise
         * it would change the values inadvertently, which was what was happening before I figured out that issue.
         * @param velocity This vertex's velocity vector
         * @param direction This vertex's direction vector
         * @param accBoolean Either a 0 or 1, indicating whether or not the vehicle is accelerating
         */
        public void updateVelocityVec(Vector velocity, Vector direction, double accBoolean) {
            Vector dirCopy = direction.copyVector();
            dirCopy.multOne(accBoolean);
            velocity.addVector(dirCopy);
            velocity.multOne(friction);
        }

        /**
         * This method updates the position vector simply by adding the change in velocity to the position.
         * @param position This vertex's position vector
         * @param velocity This vertex's velocity vector
         */
        public void updatePositionVec(Vector position, Vector velocity) {
            position.addVector(velocity);
        }

        /**
         * A method used to create a copy of this vertex.
         * @return a copy of the current vertex
         */
        public Vertex copyVertex() {
            return new Vertex(new Vector(direction.x, direction.y), new Vector(velocity.x, velocity.y),
                        new Vector(position.x, position.y));
        }

        /**
         * Returns the timestep of the control.
         * @return timeStep
         */
        public double getTimeStep() {
            return timeStep;
        }

        /**
         * Set the waypoint that was hit at this Vertex.
         */
        public void setWaypointHit(int waypointNum) {
            waypointHit = waypointNum;
        }

        @Override
        public String toString() {
            String str = "";
            str += "Direction: " + direction.x + ", " + direction.y + "\n";
            str += "Velocity: " + velocity.x + ", " + velocity.y + "\n";
            str += "Position: " + position.x + ", " + position.y + "\n";
            str += "Time Step: " + timeStep + "\n";

            if (control != null) {
                str += "Turn: " + control[1] + "\n";
                str += "Acceleration: " + control[0] + "\n";
            }

            return str;
        }
    }

    /**
     * Returns the last state of the vehicle.
     * @return the last state
     */
    public Vertex getLastState() {
        return last;
    }

    /**
     * Writes the controls and timestamps of the solution trajectory to a file called "controls.txt".
     * @param solution The ArrayList of all the vertices that form the solution trajectory
     */
    public static void writeControls(LinkedList<Vertex> solution) {
        try {
            FileWriter controls = new FileWriter("controls.txt", false);
            controls.write(solution.size() - 1 + "\n");
            Iterator it = solution.iterator();
            it.next(); // Skip first state

            while (it.hasNext()) {
                Vertex state = (Vertex) it.next();
                controls.write(state.control[1] + " " + state.control[0] + " "
                        + state.timeStep + " " + state.waypointHit + "\n");
            }

            controls.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Writes the states (including x and y position and angle) of the solution path to a file called "states.txt".
     * @param solution The ArrayList of all of the vertices that form the solution trajectory
     */
    public static void writeStates(LinkedList<Vertex> solution) {
        try {
            FileWriter fw = new FileWriter("states.txt", false);
            fw.write(solution.size() + "\n");
            for (int i = 0; i < solution.size(); i++) {
                fw.write(solution.get(i).position.x + " "
                        + solution.get(i).position.y + " "
                        + Math.atan2(solution.get(i).direction.y, solution.get(i).direction.x) + "\n");
            }
            fw.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Writes all of the points tried in during the run of the algorithm to a file called "treeView.txt".
     * @param points The ArrayList of all of the points tried by the RRT during it's run
     */
    public void writePointsTest(ArrayList<Vertex> points) {
        try {
            FileWriter fw = new FileWriter("treeView.txt", false);
            fw.write(points.size() + "\n");
            for (int i = points.size() - 1; i >= 0; i--) {
                fw.write(points.get(i).position.x + " "
                        + points.get(i).position.y + " "
                        + Math.atan2(points.get(i).direction.y, points.get(i).direction.x) + "\n");
            }
            fw.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * This method creates a direction vector based on the angle given to it in the parameter. No more fiddling about
     * to find the correct starting angle values, yay!
     * @param angleDegree The desired starting angle (in degrees, from 0 - 360)
     * @return A direction vector with the correct x and y values
     */
    public Vector degToDir(int angleDegree) {
        double angle = angleDegree * Math.PI / 180;
        return new Vector(Math.cos(angle), Math.sin(angle));
    }

    /**
     * The main method currently being used for testing with the new PTSP vehicle, instead of the grad student one.
     */
    public static void mainPTSP() {
        RRT rrt = new RRT();
        rrt.width = 50;
        rrt.height = 50;
        rrt.map = "___#_"
            + "_#_#_"
            + "_#_#_"
            + "_#___"
            + "_####";
        rrt.readMap(rrt.map);
        rrt.root = new Vertex(rrt.degToDir(90), rrt.blankVect(), new Vector(5, 5));
        rrt.root.parent = null;
        rrt.points.add(rrt.root);
        rrt.goal = new Vertex(new Vector(0, 0), rrt.blankVect(), new Vector(45, 45));
        rrt.goal.parent = null;
        rrt.r = new Random();
        LinkedList<Vertex> states = rrt.generateRRT(100000);
        /*r.writePointsTest(r.points);
        System.out.println("Total states: " + states.size());
        for (int i = 0; i < states.size(); i++) {
            System.out.println(states.get(i).position.x + " "
                    + states.get(i).position.y + " "
                    + Math.atan2(states.get(i).direction.y, states.get(i).direction.x));
        }*/
        writeControls(states);
        writeStates(states);
    }

    /**
     * The main method. Duh.
     * @param args None
     */
    public static void main(String[] args) {
        mainPTSP();
    }
}
