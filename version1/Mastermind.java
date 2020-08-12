import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.math.RoundingMode;
import java.util.Scanner;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Iterator;
import java.text.DecimalFormat;
import ptspSolver.TreePTSP;
import dirt.DIRT;
import dirt.DIRT.State;
import dirt.Vector;

/**
 * Program that solves the Physical Traveling Salesman Problem.
 * @author Bryan McKenney, Lucas Guerrette
 * @version 1.5
 */
public class Mastermind {
    private double maxSpeed;
    private double[] initialDir;
    private double[][] coords;
    private double[][] times;
    private long seed;
    private int worldWidth;
    private int worldHeight;
    private int iterations;
    private int edgesPerIteration;
    private String worldMap;
    private String fileName;
    private double runtime;
    private double bestTime;
    private int run;
    private int totalRuns;
    private HashMap<String, LinkedList<State>> cacheSuccess;
    private HashMap<String, Integer> cacheFailure;
    private int savedRuns = 0;
    private int legTests = 0;
    private int successes = 0;
    private int fails = 0;
    private int skippedRuns = 0;

    /**
     * Given a PTSP file, write a sequence of vehicle controls to a file.
     * @param args Name of PTSP file, node limit for PTSP Solver, iterations for motion planner,
     * edge candidates per iteration for motion planner
     */
    public static void main(String[] args) {
        new Mastermind(args);
    }

    /**
     * Constructor that does the main method's dirty work.
     * @param args Same as above
     */
    public Mastermind(String[] args) {
        long startTime = System.nanoTime();
        
        // If five command line arguments were passed, solve the problem file
        if (args.length > 4) {
            // Get parameters from standard in
            String filename = args[0];
            fileName = filename.split("\\.")[0];
            long nodeLimit = Long.parseLong(args[1]);
            iterations = Integer.parseInt(args[2]);
            edgesPerIteration = Integer.parseInt(args[3]);
            seed = Long.parseLong(args[4]);

            // Initialize field variables from file
            parseFile(filename);

            // Set up to find the best solution from the main loop
            LinkedList<java.awt.Taskbar.State> bestControls = new LinkedList<>();
            bestTime = Double.MAX_VALUE;

            // Run progress variables
            run = 0;
            totalRuns = (times.length + 1) * (times.length - 2) * 48 + (times.length + 1);

            // Initialize caches
            cacheSuccess = new HashMap<>();
            cacheFailure = new HashMap<>();

            // Main loop (loop until time matrix is filled with updated times, and then keep looping and pray to RNGesus)
            for (int i = 0; i < times.length + 1; i++) {
                System.out.println("\nMAIN LOOP ITERATION " + (i + 1) + "\n");

                // Initialize list to hold all controls for the solution to this problem
                LinkedList<javax.swing.plaf.nimbus.State<JComponent>> allControls = new LinkedList<>();

                // Find an ordering for the waypoints
                TreePTSP ptspSolver = new TreePTSP(coords, times, nodeLimit);
                int[] ordering = ptspSolver.getOrdering();
                double[][] orderedCoords = ptspSolver.getSolution();

                //System.out.println(Arrays.deepToString(times));
                System.out.println(Arrays.toString(ordering));

                // Find controls to reach those waypoints
                State root = new State(new Vector(initialDir[0], initialDir[1]), new Vector(0, 0), new Vector(orderedCoords[0][0], orderedCoords[0][1]));
                Vector currPos;
                Vector firstGoalPos = new Vector(orderedCoords[1][0], orderedCoords[1][1]);
                Vector secondGoalPos;
                Vector[] velRange = new Vector[3];
                Vector[] headings;
                State x, x2;
                State[] bestStates;
                LinkedList<java.awt.Taskbar.State> controls;
                double totalTime = 0;

                // Run DIRT from node 0 to 1 to find the max velocity and direction for node 1
                if (times.length > 2) {
                    System.out.println("\nDIRT Run " + (++run) + "/" + totalRuns);
                } else {
                    System.out.println("\nDIRT Run 1/1"); // One-waypoint problems only run DIRT once
                }

                System.out.println("Running DIRT between start state and first waypoint...");
                DIRT.goal = new State(new Vector(0, 0), new Vector(0, 0), firstGoalPos);
                DIRT motionPlanner = new DIRT(root, worldWidth, worldHeight, worldMap, iterations, edgesPerIteration, false, seed);
                x = motionPlanner.solve();

                if (x == null) {
                    break;
                }
                
                velRange[0] = new Vector(0, 0);
                velRange[1] = new Vector(x.velocity.x / 2, x.velocity.y / 2);
                velRange[2] = x.velocity;

                // Run DIRT as many times as it takes to hit all the waypoints
                for (int j = 1; j < orderedCoords.length - 1; j++) {
                    bestStates = new State[2];
                    controls = new LinkedList<>();
                    currPos = new Vector(orderedCoords[j - 1][0], orderedCoords[j - 1][1]);
                    firstGoalPos = new Vector(orderedCoords[j][0], orderedCoords[j][1]);
                    secondGoalPos = new Vector(orderedCoords[j + 1][0], orderedCoords[j + 1][1]);

                    // Calculate headings
                    double angle = Math.atan2(secondGoalPos.y - currPos.y, secondGoalPos.x - currPos.x) * 180 / Math.PI;
                    headings = new Vector[]{degToDir(angle), degToDir(angle + 45), degToDir(angle + 90), degToDir(angle + 135),
                        degToDir(angle + 180), degToDir(angle + 225), degToDir(angle + 270), degToDir(angle + 315)};
                    
                    int from = ordering[j - 1];
                    int to = ordering[j];
                    int next = ordering[j + 1];
                    System.out.println("\nRunning DIRT twice from waypoint " + from + " to waypoint " + next + "...");

                    // Try all possible combinations of velocities and headings to hit the first goal at and choose the
                    // one that allows the vehicle to get the the second goal in the fastest time
                    for (int v = 0; v < velRange.length; v++) {
                        for (int h = 0; h < headings.length; h++) {
                            System.out.println("\nIteration: " + ((v * 8) + h + 1));
                            System.out.println("Testing velocity " + velRange[v] + " and heading " + headings[h] + "...");

                            // Set up for the next run of DIRT
                            DIRT.goal = new State(headings[h], velRange[v], firstGoalPos);
                            x = runLeg(root, true, from, to);
                            
                            // If no trajectory was found, skip to the next iteration
                            if (x == null) {
                                continue;
                            }

                            // Set up for the next run of DIRT
                            State secondRoot = x.copyState();
                            DIRT.goal = new State(new Vector(0, 0), new Vector(0, 0), secondGoalPos);
                            x2 = runLeg(secondRoot, false, to, next);
                            
                            // If no trajectory was found, skip to the next iteration
                            if (x2 == null) {
                                continue;
                            }

                            // Update the best states found so far
                            if (bestStates[0] == null || x.cost + x2.cost < bestStates[0].cost + bestStates[1].cost) {
                                bestStates[0] = x;
                                bestStates[1] = x2;
                            }
                        }
                    }

                    // Set the root for the next iteration to be where this one left off
                    root = bestStates[0];

                    // Alert the user if no trajectory was found
                    if (root == null) {
                        System.out.println("No possible trajectory found from waypoint " + from + " to waypoint " + to + ". Better luck next time!");
                        times[from][to] = 10000; // Set the time to a high value for failure
                        break;
                    }

                    // Update the velocity range for the next iteration based on the velocity that this leg ended at
                    Vector lastVelocity = bestStates[1].velocity;
                    velRange[1] = new Vector(lastVelocity.x / 2, lastVelocity.y / 2);
                    velRange[2] = new Vector(lastVelocity.x, lastVelocity.y);

                    // Add all of the ancestor states of the solution to a LinkedList for controls
                    x = root;

                    while (x != null) {
                        controls.addFirst(x);
                        x = x.parent;
                    }

                    // Remove the root unless it is the first one, because otherwise it will be a duplicate of the last end state
                    if (j != 1) {
                        controls.removeFirst();
                    }

                    // Find the total control time and use it to update the time between corresponding nodes in the time matrix
                    double t = controls.getLast().cost;
                    times[from][to] = t;
                    totalTime += t;
                    System.out.println(from + "-" + to + " time: " + t);

                    // Add these controls to allControls
                    if (totalTime < bestTime) {
                        allControls.addAll(controls);
                    } else {
                        break; // Branch and bound
                    }

                    // Update the last state with what waypoint it ended on
                    root.setWaypointHit(to);

                    // In the last iteration, get to the final waypoint
                    if (j == orderedCoords.length - 2) {
                        controls = new LinkedList<>();
                        x = bestStates[1];

                        while (x != null) {
                            controls.addFirst(x);
                            x = x.parent;
                        }
    
                        // Remove the root because it is a duplicate of the last end state
                        controls.removeFirst();
    
                        // Find the total control time and use it to update the time between corresponding nodes in the time matrix
                        t = controls.getLast().cost;
                        times[to][next] = t;
                        totalTime += t;
                        System.out.println(to + "-" + next + " time: " + t);
    
                        // Add these controls to allControls
                        if (totalTime < bestTime) {
                            allControls.addAll(controls);
                            bestControls = allControls;
                            bestTime = totalTime;

                            // Light up the last waypoint
                            bestControls.getLast().setWaypointHit(next);
                        }
                    }
                }

                // Handle one-waypoint PTSP
                if (times.length == 2) {
                    controls = new LinkedList<>();
    
                    while (x != null) {
                        controls.addFirst(x);
                        x = x.parent;
                    }
    
                    // Add these controls to allControls
                    bestControls.addAll(controls);
                    
                    bestTime = bestControls.getLast().cost;
                    // Update the last state with what waypoint it ended on
                    bestControls.getLast().setWaypointHit(1);

                    successes++;

                    // No need to run this one more than once
                    break;
                }
            }

            System.out.println("\nCached leg lists: " + cacheSuccess.size());
            System.out.println("Cached failed legs: " + cacheFailure.size());
            System.out.println("\nLegs tested: " + legTests);
            System.out.println("Legs reused: " + savedRuns);
            System.out.println("Legs skipped: " + skippedRuns);
            System.out.println("\nTotal successful DIRT runs: " + successes);
            System.out.println("Total failed DIRT runs: " + fails);

            // Write all controls to a file
            DIRT.writeControls(bestControls);

            // Make runs more distinguishable by printing DONE and whitespace at the end of execution
            System.out.println("\nDONE\n");
        } else {
            System.out.println("Not enough arguments.");
        }
        
        long endTime = System.nanoTime();
        runtime = 1.0 * (endTime - startTime) / Math.pow(10, 9);
        writeData();
    }

    /**
     * Write the data to a csv file.
     */
    private void writeData() {
        try {
            boolean justCreated = false;
            File test = new File(fileName + ".csv");

            if (!test.exists()) {
                justCreated = true;
            }
            
            FileWriter data = new FileWriter(test, true);

            // If this file was just created, add the first line for the value names
            if (justCreated) {
                data.write("\"Solution Cost\",\"Total CPU Time\",\"# of Calls to Motion Planner\",\"# of Runs Reused\",\"# of Runs Skipped\",\"# of Orderings\"\n");
            }

            // Append values for DIRT and RRT runs on this problem
            data.write(bestTime + "," + runtime + "," + (successes + fails) + "," + savedRuns + "," + skippedRuns + "," + (times.length + 1) + "\n");
            data.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Run a leg of DIRT and cache it or use a previously cached success (or skip based on a previously cached failure).
     */
    private State runLeg(State root, boolean fullGoalCheck, int start, int end) {
        DIRT motionPlanner = new DIRT(root, worldWidth, worldHeight, worldMap, iterations, edgesPerIteration, fullGoalCheck, seed);
        System.out.println("\nDIRT Run " + (++run) + "/" + totalRuns);
        System.out.println("Running DIRT from waypoint " + start + " to waypoint " + end + "...");
        
        // Generate a hash key based on the current state and goal and see if there's an already-cached trajectory that could work here
        String key = generateHashKey(start, end, root.direction, root.velocity, DIRT.goal.direction, DIRT.goal.velocity);
        LinkedList<State> legs = cacheSuccess.get(key);
        Integer fail = cacheFailure.get(key);
        State x = null;

        // If such a trajectory (or multiple) exists, test it to see if it hits the goal (and avoids obstacles) from this state
        if (legs != null) {
            Iterator<State> it = legs.iterator();
            
            while (it.hasNext()) {
                State legTest = motionPlanner.testLeg(it.next());
                legTests++;

                // Set x to be the best of the successful leg tests
                if (legTest != null && (x == null || legTest.cost < x.cost)) {
                    x = legTest;
                }
            }
        } else if (fail != null) { // Skip running DIRT where it has failed before
            System.out.println("Did not run DIRT, gave up hope because of past failures in life.");
            skippedRuns++;
            return null;
        }

        // If there was no cached trajectory that fits the situation or it failed, run DIRT
        if (x == null) {
            // Go to the next waypoint, caring only about position
            x = motionPlanner.solve();

            // If DIRT found a successful trajectory, cache it
            if (x != null) {
                successes++;

                if (legs == null) { // Create new list of legs with x in it
                    legs = new LinkedList<State>();
                    legs.push(x);
                    cacheSuccess.put(key, legs);
                } else { // Add x to existing list of legs
                    legs.push(x);
                }
            } else {
                fails++;
                cacheFailure.put(key, 0);
            }
        } else {
            System.out.println("Did not run DIRT, found a cached leg that worked.");
            savedRuns++;
        }

        return x;
    }

    /**
     * Generates a key for the hash map using the given parameter values. -------------------------------------------------------------
     */
    private String generateHashKey(int from, int to, Vector direction, Vector velocity, Vector heading, Vector velRange) {
        // Rounding velocities (may have to tweak if completely inaccurate)
        int velocityX = (int) Math.round(velocity.x);
        int velocityY = (int) Math.round(velocity.y);
        int velRangeX = (int) Math.round(velRange.x);
        int velRangeY = (int) Math.round(velRange.y);
        
        // Rounding headings (may have to tweak if completely inaccurate)
        DecimalFormat df = new DecimalFormat("#.#");
        df.setRoundingMode(RoundingMode.CEILING);
        String dirX = df.format(direction.x);
        String dirY = df.format(direction.y);
        String headX = df.format(heading.x);
        String headY = df.format(heading.y);

        // Return the hash key in the correct format
        return "" + from + to + dirX + dirY + velocityX + velocityY + headX + headY + velRangeX + velRangeY;
    }

    /**
     * This method creates a direction vector based on the angle given to it in the parameter. No more fiddling about -----------------
     * to find the correct starting angle values, yay!
     * @param angleDegree The desired starting angle (in degrees, from 0 - 360)
     * @return A direction vector with the correct x and y values
     */
    private Vector degToDir(double angleDegree) {
        double angle = angleDegree * Math.PI / 180;
        return new Vector(Math.cos(angle), Math.sin(angle));
    }

    /**
     * Parse a file containing a PTSP and set times to the matrix representing that problem. ------------------------------------------
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
                    worldMap = firstRow + "\n";

                    // Add the rest of the lines to the map string
                    for (int i = 1; i < mapHeight; i++) {
                        worldMap += scnr.nextLine();

                        if (i != mapHeight - 1) {
                            worldMap += "\n";
                        }
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

                    // Find the max speed of the vehicle on this map
                    setMaxSpeed();

                    // Fill times based on the values in coords and the max speed
                    coordsToMatrix();
                }
            }

            scnr.close();
        } catch (FileNotFoundException e) {
            System.out.println("File not found.");
        }
    }

    /**
     * Set the max speed of the vehicle depending on the map size
     */
    private void setMaxSpeed() {
        // Find the distance between the lower left and upper right corners of the map
        double distance = Math.sqrt(Math.pow(worldWidth, 2) + Math.pow(worldHeight, 2));
        
        // Solve for the final velocity going between those points with constant max acceleration
        maxSpeed = Math.sqrt(2 * distance);
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
                    double t = dis / maxSpeed;
            
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