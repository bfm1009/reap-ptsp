import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.Iterator;
import ptspSolver.TreePTSP;
import necromancer.Headings;
import motionPlanner.PTSPVehicleMotionPlanner.RRT;
import motionPlanner.PTSPVehicleMotionPlanner.RRT.Vertex;
import motionPlanner.PTSPVehicleMotionPlanner.Vector;

/**
 * Program that solves the Physical Traveling Salesman Problem.
 * @author Bryan McKenney, Lucas Guerrette
 * @version 0.5
 */
public class Mastermind {
    private final double HALLUCINATED_SPEED = 20;
    private double[] initialDir;
    private double[][] coords;
    private double[][] times;
    private int worldWidth;
    private int worldHeight;
    private String worldMap;
    private LinkedList<Vertex> allControls;
    private double bestTime;
    private LinkedList<Vertex> bestControls;

    /**
     * Given a PTSP file, write a sequence of vehicle controls to a file.
     * @param args Name of PTSP file, node limit for PTSP Solver, max iterations for motion planner
     */
    public static void main(String[] args) {
        new Mastermind(args);
    }

    /**
     * Constructor that does the main method's dirty work.
     * @param args Same as above
     */
    public Mastermind(String[] args) {
        // If three command line arguments were passed, solve the problem file
        if (args.length > 2) {
            // Get parameters from standard in
            String filename = args[0];
            long nodeLimit = Long.parseLong(args[1]);
            int maxIterations = Integer.parseInt(args[2]);

            // Initialize field variables from file
            parseFile(filename);

            // Set best time to be the worst possible time (so that it can only get better)
            bestTime = Double.MAX_VALUE;

            // Main loop (loop until time matrix is filled with updated times, and then loop that many times again and pray to RNGesus)
            for (int i = 0; i < times.length * 5; i++) {
                // Find an ordering for the waypoints
                TreePTSP ptspSolver = new TreePTSP(coords, times, nodeLimit);
                int[] ordering = ptspSolver.getOrdering();
                double[][] orderedCoords = ptspSolver.getSolution();

                System.out.println(Arrays.deepToString(times));
                System.out.println(Arrays.toString(ordering));

                // Find headings for the waypoints
                Headings necromancer = new Headings(initialDir, orderedCoords);
                double[][] headings = necromancer.calcEnd();

                // Find controls to reach those headings
                Vertex root = RRT.loadPoint(headings[0]);
                Vertex goal;
                allControls = new LinkedList<Vertex>();
                double totalTime = 0;

                for (int j = 1; j < headings.length; j++) { // Run the RRT as many times as it takes to hit all the waypoints
                    goal = RRT.loadPoint(headings[j]);
                    RRT motionPlanner = new RRT(root, goal, worldWidth, worldHeight, worldMap);
                    LinkedList<Vertex> controls = motionPlanner.generateRRT(maxIterations);

                    // Remove the root unless it is the first one, because otherwise it will be a duplicate of the last end state
                    if (j != 1) {
                        controls.remove();
                    }

                    // Add up the time that each control runs for and use it to update the time between corresponding nodes in the time matrix
                    double t = 0;
                    Iterator it = controls.iterator();
                    int from = ordering[j - 1];
                    int to = ordering[j];

                    while (it.hasNext()) {
                        Vertex control = (Vertex) it.next();
                        t += control.getTimeStep();
                    }

                    times[from][to] = t;
                    System.out.println(from + "-" + to + " time: " + t);

                    // Increase the total time and add these controls to allControls
                    totalTime += t;
                    allControls.addAll(controls);

                    // For the next iteration of the inner loop, set the root to be where this set of controls left the vehicle
                    root = motionPlanner.getLastState();

                    // Update the last state with what waypoint it ended on
                    root.setWaypointHit(to);
                }

                // If this set of controls is better than the best found so far, replace the best one
                if (totalTime < bestTime) {
                    bestControls = allControls;
                    bestTime = totalTime;
                }
            }

            // Write all controls to a file (as well as states, for debugging)
            RRT.writeControls(bestControls);
            RRT.writeStates(bestControls);

            // Make runs more distinguishable by printing DONE and whitespace at the end of execution
            System.out.println("\nDONE\n");
        } else {
            System.out.println("Please provide filename, node limit (-1 for none), and max iterations"
                    + " as command line arguments.");
        }
    }

    /**
     * Parse a file containing a PTSP and set times to the matrix representing that problem.
     * @param filename The file to parse
     */
    private void parseFile(String filename) {
        times = new double[][]{};

        try {
            File file = new File(filename);
            Scanner scnr = new Scanner(file);

            while (scnr.hasNext()) {
                String token = scnr.next();

                // Set the dimensions of the world
                if (token.equals("WORLD_DIMENSIONS:")) {
                    worldWidth = scnr.nextInt();
                    worldHeight = scnr.nextInt();
                }

                // Set the dimensions of the matrix
                else if (token.equals("NUM_WAYPOINTS:")) {
                    int d = scnr.nextInt() + 1; // +1 because the initial position will be counted as a node here
                    times = new double[d][d];
                }

                // Set the initial direction of the vehicle
                else if (token.equals("INITIAL_DIR:")) {
                    double degrees = scnr.nextDouble();
                    double radians = Math.toRadians(degrees);
                    double x = Math.cos(radians);
                    double y = Math.sin(radians);
                    initialDir = new double[]{x, y};
                }

                // Set the initial position of the vehicle to be node 0
                else if (token.equals("INITIAL_POS:")) {
                    double x = scnr.nextDouble();
                    double y = scnr.nextDouble();
                    coords = new double[times.length][2];
                    coords[0] = new double[]{x, y};
                }

                // Read in world map
                else if (token.equals("MAP")) {
                    // Find the number of lines the map is based on the length of the first line
                    scnr.nextLine(); // Consume the newline
                    String firstRow = scnr.nextLine();
                    int scaleFactor = worldWidth / firstRow.length();
                    int mapHeight = worldHeight / scaleFactor;

                    // Set the map string equal to the first line
                    worldMap = firstRow;

                    // Add the rest of the lines to the map string
                    for (int i = 1; i < mapHeight; i++) {
                        worldMap += scnr.nextLine();
                    }
                }

                // Start reading in values from coordinate format
                else if (token.equals("WAYPOINTS")) {
                    int i = 1; // Start at 1 because coords[0] has already been filled

                    // Read the coordinates into coords
                    while (scnr.hasNextDouble()) {
                        scnr.nextDouble(); // Skip number indicating node number
                        coords[i][0] = scnr.nextDouble();
                        coords[i][1] = scnr.nextDouble();
                        scnr.nextDouble(); // Skip number indicating node radius
                        i++;
                    }

                    // Fill times based on the values in coords
                    coordsToMatrix();
                }
            }

            scnr.close();
        } catch (FileNotFoundException e) {
            System.out.println("File not found.");
        }
    }

    /**
     * Use 2D coordinates to set the values in the time matrix.
     */
    private void coordsToMatrix() {
        // Calculate the times between the coordinates to fill times
        for (int row = 0; row < coords.length; row++) {
            for (int col = 0; col < coords.length; col++) {
                if (row != col) {
                    // Use the distance formula to find distance between the two nodes
                    double dis = Math.sqrt(Math.pow(coords[row][0] - coords[col][0], 2)
                            + Math.pow(coords[row][1] - coords[col][1], 2));

                    // Calculate the supposed time between the two nodes
                    double t = dis / HALLUCINATED_SPEED;

                    // Put that time in both corresponding places in the matrix
                    times[row][col] = t;
                    times[col][row] = t;
                } else {
                    times[row][col] = 0; // Set a node's travel time to itself equal to 0
                }
            }
        }
    }
}