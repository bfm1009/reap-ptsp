package dirt;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.Random;
import java.util.Scanner;

/**
 * A motion planner that is way better than RRT.
 * @author Bryan McKenney, Lucas Guerrette
 * @version 0
 */
public class DIRT {
    // Global constant
    final static double WAYPOINT_RADIUS = 3;

    // Field variables
    private LinkedList<State> tree;
    private ArrayList<int[]> blockedSpaces;
    private State root;
    public static State goal;
    private int worldWidth;
    private int worldHeight;
    private int iterations;
    private int edgesPerIteration;
    private int scaleFactor;
    private boolean firstSol = false; //If true, stops after first solution, else keeps going to iteration limit
    private Random r;
    private static boolean fullGoalCheck;

    /**
     * Constructor.
     * @param root The starting state of the vehicle
     * @param worldWidth The width of the map
     * @param worldHeight The height of the map
     * @param map A string representation of the map
     * @param iterations The number of iterations to run the algorithm
     * @param edgesPerIteration The number of edges to expand per iteration
     * @param fullGoalCheck Whether to check velocity and direction of the goal state or not
     * @param seed The random seed
     */
    public DIRT(State root, int worldWidth, int worldHeight, String map, int iterations, int edgesPerIteration, boolean fullGoalCheck, long seed) {
        tree = new LinkedList<State>();
        root.parent = null;
        root.radius = 0;
        tree.add(root);
        this.root = root;
        this.worldWidth = worldWidth;
        this.worldHeight = worldHeight;
        blockedSpaces = readMap(map);
        this.iterations = iterations;
        this.edgesPerIteration = edgesPerIteration;
        DIRT.fullGoalCheck = fullGoalCheck;
        r = new Random(seed);
    }

    /**
     * Overloaded constructor (initialize DIRT with random seed instead of specifying it).
     */
    public DIRT(State root, int worldWidth, int worldHeight, String map, int iterations, int edgesPerIteration, boolean fullGoalCheck) {
        tree = new LinkedList<State>();
        root.parent = null;
        root.radius = 0;
        tree.add(root);
        this.root = root;
        this.worldWidth = worldWidth;
        this.worldHeight = worldHeight;
        blockedSpaces = readMap(map);
        this.iterations = iterations;
        this.edgesPerIteration = edgesPerIteration;
        DIRT.fullGoalCheck = fullGoalCheck;
        r = new Random();
    }

    /**
     * This method reads in the map from the given text file. The map string gets created in the main method, and is
     * fed into here to find the blocked spaces.
     * @param map A string containing the map of the world
     * @return An ArrayList of integer arrays, with each array containing the x and y coordinates of the bottom left
     * point of each obstacle
     */
    public ArrayList<int[]> readMap(String map) {
        blockedSpaces = new ArrayList<>();
        Scanner sc = new Scanner(map);
        String line = sc.nextLine();
        scaleFactor = worldWidth / line.length();
        int height = worldHeight / scaleFactor;
        while (height > 0) {
            for (int i = 0; i < line.length(); i++) {
                if (line.charAt(i) == '#') {
                    blockedSpaces.add(new int[] {i, height - 1});
                }
            }
            height--;
            if (height > 0) {
                line = sc.nextLine();
            }
        }
        sc.close();
        return blockedSpaces;
    }

    /**
     * Find the optimal trajectory between the two states.
     * @return The trajectory (end state)
     */
    public State solve() {
        State sol = null;
        State xNew = root;
        State xSel;

        for (int i = 0; i < iterations; i++) {

            // If firstSol is true, DIRT stops after it finds its first solution
            if (firstSol && sol != null) {
                System.out.println("Solution found, stopping loop");
                break;
            }

            // Select a state
            if (xNew != null && xNew.parent != null && xNew.heuristicVal < xNew.parent.heuristicVal) {
                xSel = xNew;
            } else {
                xSel = DIRselection();
            }

            // If there are no edge candidates, blossom from the state to find some
            if (xSel.Ecand.size() == 0) {
                blossom(xSel); // Chooses edge candidates (Ecand) of the state
            }

            // While there are edge candidates left, extend the tree
            while (xSel.Ecand.size() > 0) {
                // Find the best state to go to from xSel (uses the shortest controls)
                xNew = bestNextState(xSel);

                // If the state would not make the solution worse, add it to the tree
                if (xNew != null && (sol == null || xNew.pathQuality <= sol.cost)) {
                    // Extend the tree with xNew
                    tree.add(xNew);
                    xNew.parent = xSel;

                    // Update the DIRs (size of the radii) of all the states in the tree that
                    // are up to the same distance away from xNew as xNew is from its parent, xSel
                    if (sol != null) {
                        updateDIRpruning(xSel, xNew);
                    } else {
                        updateDominanceRegions(xSel, xNew);
                    }
                    // If the cost of the trajectory ending in xNew is less than the solution cost,
                    // update the solution
                    if (xNew.heuristicVal == 0 && (sol == null || xNew.cost < sol.cost)) {
                        sol = xNew;
                        System.out.println("Solution found! Total time: " + sol.cost);
                    }
                    
                    break;
                } else {
                    xNew = null;
                }
            }
        }

        // Display if no solution was found
        if (sol == null) {
            System.out.println("No trajectory found.");
        }

        // Print out size of tree
        System.out.println("Tree size: " + tree.size());

        // Returns the solution state
        return sol;
    }

    /**
     * Return a random state.
     * @return A state
     */
    private State DIRselection() {
        LinkedList<State> Xcand = new LinkedList<State>();
        Iterator<State> it = tree.iterator();
        State x;

        // Choose a random point
        Vector xRand = randomSample();

        // Find all the states in the tree that are nearby that point
        while (it.hasNext()) {
            x = (State) it.next();

            if (x.inDIR(xRand)) {
                Xcand.add(x);
            }
        }

        // If there aren't any nearby states (the point is in an unexplored region), choose the state
        // in the tree that is closest to the point and find all tree states nearby that instead
        if (Xcand.size() == 0) {
            Vector xClosest = null;
            double shortestDis = Double.MAX_VALUE;
            double dis;

            // Find the closest tree state to the point
            it = tree.iterator();

            while (it.hasNext()) {
                x = (State) it.next();
                dis = distance(xRand.x, xRand.y, x.position.x, x.position.y);

                if (dis < shortestDis) {
                    shortestDis = dis;
                    xClosest = new Vector(x.position.x, x.position.y);
                }
            }

            // Find all tree states nearby that state
            it = tree.iterator();

            while (it.hasNext()) {
                x = (State) it.next();
    
                if (x.inDIR(xClosest)) {
                    Xcand.add(x);
                }
            }
        }

        // Randomly choose one of the nearby states and return it
        int randIndex = r.nextInt(Xcand.size());
        return Xcand.get(randIndex);
    }

    /**
     * Choose a random point in the state space.
     * @return The point
     */
    private Vector randomSample() {
        double x, y;

        // Random chance of returning the goal as the sample state
        if (r.nextInt(100) <= 5) {
            return new Vector(goal.position.x, goal.position.y);
        }

        // Get initial random values for x and y
        x = r.nextDouble() * worldWidth;
        y = r.nextDouble() * worldHeight;

        // Re-roll those values until the point isn't in an obstacle
        while (posCollision(x, y)) {
            x = r.nextDouble() * worldWidth;
            y = r.nextDouble() * worldHeight;
        }

        return new Vector(x, y);
    }

    /**
     * Updates the radius (AKA dominance region) of all of the States in the tree.
     * @param tree the tree
     * @param xSelected xNew's parent State
     * @param xNew the State being assigned a radius
     */
    public void updateDominanceRegions(State xSelected, State xNew) {
        LinkedList<State> xUp = new LinkedList<>();
        Iterator<State> iTree = tree.iterator(); 
        double distance = xSelected.distPoints(xNew);
        // Goes through the tree and adds a State to xUp if the distance is less than the distance between 
        // xNew and its parent State xSelected
        while (iTree.hasNext()) { 
            State x = (State) iTree.next();
            if (x.distPoints(xNew) <= distance) {
                xUp.add(x); 
            }
        }
        // Updates the radii of the DIRs of all the x in xUp if they have a greater path cost than xNew to be smaller,
        // as well as finding the maximum possible radius of xNew's DIR
        Iterator<State> ixUp = xUp.iterator();
        double maxDT = 0;
        while (ixUp.hasNext()) {
            State x = (State) ixUp.next();
            if (x.pathQuality > xNew.pathQuality) {
                x.radius = x.distPoints(xNew);
                if (xNew.distPoints(x) > maxDT) {
                    maxDT = xNew.distPoints(x);
                }
            }
        }
        // Sets the radius of xNew's DIR to whatever maximum value it found in the above loop
        xNew.radius = maxDT;
        if (maxDT > root.radius) {
            root.radius = maxDT;
        }
    }

    private State bestNextState(State xSelected) {
        double[] uBest = null;
        State xBest = null;
        Iterator<double[]> iControls = xSelected.Ecand.iterator();
        while (iControls.hasNext()) {
            double[] u = iControls.next();
            State xProp = propagate(xSelected, u);
            if (xProp != null) {
               if (xBest == null || xProp.pathQuality < xBest.pathQuality) {
                   xBest = xProp;
                   uBest = u;
                }
            }
        }
        if (xBest != null) {
            xBest.controls = uBest; // Might need to clear Ecand for xBest, probable spot for bugs
            xSelected.Ecand.remove(uBest);
            xBest.Ecand.clear();
        } else {
            xSelected.Ecand.clear();
        }
        return xBest;
    }

    private State propagate(State xSelected, double[] u) {
        State xCopy = xSelected.copyState();
        for (double i = 0; i < u[2]; i += 0.05) {
            xCopy.updateDirectionVec(xCopy.direction, u[0]);
            xCopy.updateVelocityVec(xCopy.velocity, xCopy.direction, u[1]);
            xCopy.updatePositionVec(xCopy.position, xCopy.velocity);
            if (posCollision(xCopy.position.x, xCopy.position.y)) {
                return null;
            }
        }
        xCopy.controls = u;
        xCopy.setParent(xSelected);
        return xCopy;
    }

    /**
     * Test a pre-generated set of controls starting from the root and see if they
     * get the vehicle to the goal state.
     * @param leg The end state of the trajectory to test
     */
    public State testLeg(State leg) {
        LinkedList<State> traj = new LinkedList<>();
        Iterator<State> it;
        State x = root;

        // Get full trajectory from leg end state
        while (leg != null) {
            traj.addFirst(leg);
            leg = leg.parent;
        }

        traj.removeFirst();
        it = traj.iterator();

        // Iterate through trajectory, applying controls
        while (it.hasNext()) {
            x = propagate(x, it.next().controls);

            // If the vehicle hits an obstacle, return null
            if (x == null) {
                return null;
            }
        }

        // Calculate the distance from the current state to the goal
        double dis = distance(x, goal);
        double diffDir = Math.abs(x.getAngle() - goal.getAngle());
        double diffVelX = Math.abs(goal.velocity.x - x.velocity.x);
        double diffVelY = Math.abs(goal.velocity.y - x.velocity.y);

        // Return the final state if it is near to the waypoint state, or null otherwise
        if (dis <= WAYPOINT_RADIUS && (!fullGoalCheck || (diffDir <= 0.5 && diffVelX <= 0.5 && diffVelY <= 0.5))) {
            return x;
        } else {
            return null;
        }
    }

    /**
     * Takes two doubles, representing the coordinates of a vertex, and test to see if it is out of bounds. This is
     * mainly used during calculation of the trajectory, before an end point is created.
     * @param x the current x position being tested
     * @param y the current y position being tested
     * @return true if there is a collision, false otherwise
     */
    private boolean posCollision(double x, double y) {
        if (x <= 0 || x >= worldWidth || y <= 0 || y >= worldHeight) {
            return true;
        }
        for (int i = 0; i < blockedSpaces.size(); i++) {
            if (x <= (blockedSpaces.get(i)[0] * scaleFactor) + scaleFactor && x >= (blockedSpaces.get(i)[0] * scaleFactor)) {
                if (y <= (blockedSpaces.get(i)[1] * scaleFactor) + scaleFactor && y >= (blockedSpaces.get(i)[1] * scaleFactor)) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Given a state, find random controls (edge candidates) from it.
     * @param x The state to blossom from
     * @return Edge candidate(s)
     */
    private void blossom(State x) {
        LinkedList<double[]> edgeCands = new LinkedList<double[]>();

        if (x.expanded) {
            // Add a single control to edge candidates
            edgeCands.add(randomControl());
        } else {
            // Mark that this state has been expanded
            x.expanded = true;
            
            // Add a number of controls to edge candidates equal to the predetermined edges per iteration
            for (int i = 0; i < edgesPerIteration; i++) {
                edgeCands.add(randomControl());
            }
        }

        // Update the edge candidates in state x
        x.Ecand = edgeCands;
    }

    /**
     * Return a random control.
     * @return A control
     */
    private double[] randomControl() {
        double turn = (r.nextInt(3) - 1) * r.nextDouble();
        double acc = r.nextDouble() / 10;
        double time = r.nextDouble() / 2 + 0.05;
        return new double[] {turn, acc, time};
        
    }

    /**
     * Update dominance regions and then prune any states that are inside of another state.
     * @param xSelected xNew's parent state
     * @param xNew The state being assigned a radius
     */
    private void updateDIRpruning(State xSelected, State xNew) {
        updateDominanceRegions(xSelected, xNew);

        //LinkedList<State> hitList = new LinkedList<State>();
        Iterator<State> itX = tree.iterator();
        Iterator<State> itX2;
        State x;
        State x2;

        // Collect states to prune
        while (itX.hasNext()) {
            x = (State) itX.next();

            if (x != root) {
                itX2 = tree.iterator();

                while (itX2.hasNext()) {
                    x2 = (State) itX2.next();

                    // If x is entirely contained within x2, remove it from the tree
                    if (x.radius + distance(x, x2) < x2.radius) {
                        itX.remove();
                        //markForDeath(hitList, x);
                        break;
                    }
                }
            }
        }
    }

    /**
     * Return the distance between two points.
     * @param x1 x of first point
     * @param y1 y of first point
     * @param x2 x of second point
     * @param y2 y of second point
     * @return The distance
     */
    private static double distance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    }

    /**
     * Return the distance between two states.
     * @param x The first state
     * @param x2 The second state
     * @return The distance
     */
    private static double distance(State x, State x2) {
        return Math.sqrt(Math.pow(x2.position.x - x.position.x, 2) +
            Math.pow(x2.position.y - x.position.y, 2));
    }

    /**
     * Return the entire tree.
     * @return tree
     */
    public LinkedList<State> getTree() {
        return tree;
    }

    /**
     * Writes the controls and timestamps of the given trajectory to a file called "controls.txt".
     * @param traj Trajectory
     */
    public static void writeControls(LinkedList<State> traj) {
        try {
            FileWriter fw = new FileWriter("controls.txt", false);
            Iterator<State> it;
            State x;
            
            // Write number of controls to file
            fw.write(traj.size() - 1 + "\n");

            // Write the controls to file
            it = traj.iterator();
            it.next(); // Skip the root
            
            while (it.hasNext()) {
                x = (State) it.next();
                fw.write(x.controls[0] + " " + x.controls[1] + " " + x.controls[2] + " " + x.waypointHit + "\n");
            }

            fw.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Writes the x, y, and angle of each state in the given trajectory to a file called "states.txt".
     * @param traj Trajectory
     */
    public static void writeStates(LinkedList<State> traj) {
        try {
            FileWriter fw = new FileWriter("states.txt", false);
            Iterator<State> it;
            State x;
            
            // Write number of states to file
            fw.write(traj.size() + "\n");

            // Write the controls to file
            it = traj.iterator();
            
            while (it.hasNext()) {
                x = (State) it.next();
                fw.write(x.position.x + " " + x.position.y + " " + Math.atan2(x.direction.y, x.direction.x) + "\n");
            }

            fw.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static void writeTree(LinkedList<State> tree, LinkedList<State> traj) {
        try {
            FileWriter fw = new FileWriter("tree.txt", false);
            Iterator<State> itt;
            Iterator<State> it;
            State x;
            
            // Write number of states to file
            itt = tree.iterator();
            fw.write("TREE_NODES\n");
            while (itt.hasNext()) {
                x = (State) itt.next();
                fw.write(x.position.x + " " + x.position.y + " " + x.radius + "\n");
            }

            // Write the controls to file
            it = traj.iterator();
            fw.write("TRAJECTORY\n");
            while (it.hasNext()) {
                x = (State) it.next();
                fw.write(x.position.x + " " + x.position.y + " " + x.radius + "\n");
            }

            fw.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * This inner class defines each state. Each state has three vectors, one for direction, velocity, and position.
     * The class also contains the methods that do the calculations on each of those vectors.
     */
    public static class State {
        public Vector direction;
        public Vector velocity;
        public Vector position;
        public State parent;
        public double radius;
        public double cost; // g
        public double pathQuality; // f
        public double heuristicVal; // h
        public boolean expanded;
        public double[] controls; // The controls that led to this state
        LinkedList<double[]> Ecand; // Edge cadidates
        double friction = 0.99; // The force of friction applied to the velocity values each timestep.
        int waypointHit = 0; // The number of the waypoint hit at this state (0 for none)

        /**
         * Constructor.
         * @param direction The direction vector
         * @param velocity The velocity vector
         * @param position The position vector
         */
        public State(Vector direction, Vector velocity, Vector position) {
            this.direction = direction;
            this.velocity = velocity;
            this.position = position;
            this.expanded = false;
            Ecand = new LinkedList<double[]>();
        }

        public void setParent(State parent) {
            this.parent = parent;
            
            if (parent != null) {
                cost = parent.cost + controls[2];
            } else {
                cost = 0;
            }

            heuristicVal = h();
            pathQuality = cost + heuristicVal;
        }

        public double getAngle() {
            return Math.atan2(this.direction.y, this.direction.x);
        }

        /**
         * Heuristic function that returns the estimated time (lower bound) from the current state to the goal state.
         * @param curr The current state
         * @return The estimated time to the goal
         */
        public double h() {
            // Calculate the distance from the current state to the goal
            double dis = distance(this, goal);
            double diffDir = Math.abs(getAngle() - goal.getAngle());
            double diffVelX = Math.abs(goal.velocity.x - this.velocity.x);
            double diffVelY = Math.abs(goal.velocity.y - this.velocity.y);
            double time;

            // Return 0 if the state is in range of the goal; otherwise, return the lower-bound on time between the states
            if (dis <= WAYPOINT_RADIUS && (!fullGoalCheck || (diffDir <= 0.5 && diffVelX <= 0.5 && diffVelY <= 0.5))) {
                time = 0;
            } else if (fullGoalCheck) {
                double v = Math.sqrt(Math.pow(goal.velocity.x, 2) + Math.pow(goal.velocity.y, 2));
                double vo = Math.sqrt(Math.pow(this.velocity.x, 2) + Math.pow(this.velocity.y, 2));
                time = (distance(this, goal) * 2) / (v + vo);
            } else {
                double vo = Math.sqrt(Math.pow(this.velocity.x, 2) + Math.pow(this.velocity.y, 2));
                double underRad = 2 * -dis;
                double timePlus = (-vo + Math.sqrt(Math.pow(vo, 2) - underRad));
                double timeMinus = (-vo - Math.sqrt(Math.pow(vo, 2) - underRad));

                if (timePlus > 0) {
                    time = timePlus;
                } else {
                    time = timeMinus;
                }
            }

            return time;
        }

        /**
         * Calculates the task space distance between the current state and the state passed as a parameter.
         * @param goTo The state being measured to
         */
        private double distPoints(State goTo) {
            return Math.sqrt(Math.pow(position.x - goTo.position.x, 2) + Math.pow(position.y - goTo.position.y, 2));
        }

        /**
         * Returns whether or not the given point is within this state's DIR.
         * @param pt The point to check
         * @return Boolean
         */
        private boolean inDIR(Vector pt) {
            return distance(pt.x, pt.y, position.x, position.y) <= radius;
        }

        /**
         * This method takes this State's direction vector and an integer (either -1, 0, or 1), and changes the values
         * of the direction vector based on which way it is turning. It is SUPPOSED to be -1 for left, 1 for right, and
         * 0 for no change to direction, which it is, but I had to change the values for the lAlpha and rAlpha to get it
         * that way.
         * @param direction This State's direction vector
         * @param turn which direction to turn based on one of the three options.
         */
        public void updateDirectionVec(Vector direction, double turn) { // Vector parameter isn't necessary
            direction.multFour(Math.cos(turn), -Math.sin(turn), Math.sin(turn), Math.cos(turn));
        }

        /**
         * This method takes the direction vector and whether or not the vehicle is accelerating (0 or 1), and updates
         * the velocity vector based on those numbers. It has to make a copy of the direction vector, because otherwise
         * it would change the values inadvertently, which was what was happening before I figured out that issue.
         * @param velocity This State's velocity vector
         * @param direction This State's direction vector
         * @param acceleration Either a 0 or 1, indicating whether or not the vehicle is accelerating
         */
        public void updateVelocityVec(Vector velocity, Vector direction, double acceleration) {
            Vector dirCopy = direction.copyVector();
            dirCopy.multOne(acceleration);
            velocity.addVector(dirCopy);
            velocity.multOne(friction);
            this.velocity.x = Math.min(this.velocity.x, 2);
            this.velocity.y = Math.min(this.velocity.y, 2);

        }

        /**
         * This method updates the position vector simply by adding the change in velocity to the position.
         * @param position This State's position vector
         * @param velocity This State's velocity vector
         */
        public void updatePositionVec(Vector position, Vector velocity) {
            position.addVector(velocity);
        }

        /**
         * A method used to create a copy of this State.
         * @return a copy of the current State
         */
        public State copyState() {
            return new State(new Vector(direction.x, direction.y), new Vector(velocity.x, velocity.y),
                        new Vector(position.x, position.y));
        }

        /**
         * Set the waypoint that was hit at this State.
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

            if (controls != null) {
                str += "Turn: " + controls[0] + "\n";
                str += "Acceleration: " + controls[1] + "\n";
                str += "Time Step: " + controls[2] + "\n";
            }

            return str;
        }
    }

    /**
     * Test DIRT.
     * @param args Command line arguments
     */
    public static void main(String[] args) {
        // Make some dirt
        String map =    "##########################\n"
                      + "#__###_____####__#__###__#\n"
                      + "#__###__#___###__#__###__#\n"
                      + "#__###__#____##__#__###__#\n"
                      + "#__###__#__#__#__#_______#\n"
                      + "#__###__#__##____#__###__#\n"
                      + "#__###__#__###___#__###__#\n"
                      + "#_______#__####_____###__#\n"
                      + "##########################";
        DIRT.goal = new State(new Vector(0, 1), new Vector(0, -1), new Vector(20, 15));
        DIRT dirt = new DIRT(new State(new Vector(0, -1),
             new Vector(0, 0), new Vector(20, 75)), 260, 90, map, 100000, 7, true);
    }
}