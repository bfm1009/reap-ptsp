import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Scanner;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.Iterator;
import ptspSolver.TreePTSP;
import dirt.DIRT;
import dirt.DIRT.State;
import motionPlanner.PTSPVehicleMotionPlanner.RRT;
import motionPlanner.PTSPVehicleMotionPlanner.RRT.Vertex;

/**
 * Program that runs RRT and DIRT on the same problem.
 * @author Bryan McKenney, Lucas Guerrette
 * @version 1.0
 */
public class MasterTest {
    private double[] initialDir;
    private double[][] coords;
    private int worldWidth;
    private int worldHeight;
    private String worldMap;
    private String filename;
    private double DIRTruntime;
    private double RRTruntime;
    private double DIRTquality;
    private double RRTquality;

    /**
     * Given a PTSP file, write a sequence of vehicle controls to a file.
     * @param args Name of PTSP file, node limit for PTSP Solver, iterations for motion planner,
     * edge candidates per iteration for motion planner
     */
    public static void main(String[] args) {
        new MasterTest(args);
    }

    /**
     * Constructor that does the main method's dirty work.
     * @param args Same as above
     */
    public MasterTest(String[] args) {
        // If five command line arguments were passed, solve the problem file
        if (args.length > 4) {
            // Get parameters from standard in
            filename = args[0];
            int iterations = Integer.parseInt(args[1]);
            int edgesPerIteration = Integer.parseInt(args[2]); // This one is only for DIRT
            boolean fullGoalCheck = Boolean.parseBoolean(args[3]); // So is this one
            long seed = Long.parseLong(args[4]);

            // Initialize field variables from file
            parseFile(filename);

            // Get rid of newlines in map for RRT
            String worldMapRRT = worldMap.replace("\n", "");

            // Initialize other field variables to 0
            DIRTruntime = 0;
            RRTruntime = 0;
            DIRTquality = 0;
            RRTquality = 0;

            //---------------------------------------------- DIRT RUN ---------------------------------------------------
            System.out.println("Beginning DIRT Run...");
            
            try {
                // Find controls to reach the waypoint
                State root = new State(null, new dirt.Vector(initialDir[0], initialDir[1]), new dirt.Vector(0, 0), new dirt.Vector(coords[0][0], coords[0][1]));
                DIRT.goal = new State(null, new dirt.Vector(0, 0), new dirt.Vector(0, 0), new dirt.Vector(coords[1][0], coords[1][1]));

                long startDIRTruntime = System.nanoTime();
                DIRT motionPlanner = new DIRT(root, worldWidth, worldHeight, worldMap, iterations, edgesPerIteration, fullGoalCheck, seed);
                long endDIRTruntime = System.nanoTime();
                DIRTruntime = 1.0 * (endDIRTruntime - startDIRTruntime) / Math.pow(10, 9);
                DIRTquality = motionPlanner.getLastState().cost;
            } catch (Exception e) {
                e.printStackTrace();
                // If DIRT failed to find a solution, set the runtime and quality to be horrible
                DIRTruntime = 200;
                DIRTquality = 200;
            }
        
            System.out.println("End of DIRT Run.\n");

            //-------------------------------------------- RRT RUN ----------------------------------------------
            System.out.println("Beginning RRT Run...");

            try {
                Vertex rootRRT = new Vertex(new motionPlanner.PTSPVehicleMotionPlanner.Vector(initialDir[0], initialDir[1]),
                    new motionPlanner.PTSPVehicleMotionPlanner.Vector(0, 0), new motionPlanner.PTSPVehicleMotionPlanner.Vector(coords[0][0], coords[0][1]));
                Vertex goalRRT = new Vertex(null, null, new motionPlanner.PTSPVehicleMotionPlanner.Vector(coords[1][0], coords[1][1]));
                
                long startRRTruntime = System.nanoTime();
                RRT motionPlannerRRT = new RRT(rootRRT, goalRRT, worldWidth, worldHeight, worldMapRRT, seed);
                LinkedList<Vertex> controlsRRT = motionPlannerRRT.generateRRT(iterations);
                long endRRTruntime = System.nanoTime();
                RRTruntime = 1.0 * (endRRTruntime - startRRTruntime) / Math.pow(10, 9);

                // Add up the time that each control runs for and use it to update the time between corresponding nodes in the time matrix
                Iterator itRRT = controlsRRT.iterator();
                itRRT.next(); // Skip the root

                while (itRRT.hasNext()) {
                    Vertex control = (Vertex) itRRT.next();
                    RRTquality += control.getTimeStep();
                }
            } catch (Exception e) {
                // If RRT failed to find a solution, set the runtime and quality to be horrible
                RRTruntime = 200;
                RRTquality = 200;
            }

            System.out.println("End of RRT Run.\n");

            // Add data to csv files
            writeData();
            writeRatios();
            writeDataForScatter();
        }
    }

    /**
     * Write the data to a csv file.
     */
    private void writeData() {
        try {
            String file = filename.split("\\.")[0];
            boolean justCreated = false;
            File test = new File(file + ".csv");

            if (!test.exists()) {
                justCreated = true;
            }
            
            FileWriter data = new FileWriter(file + ".csv", true);

            // If this file was just created, add the first line for the value names
            if (justCreated) {
                data.write("\"Motion Planning Algorithm\",\"Runtime\",\"Solution Cost\"\n");
            }

            // Append values for DIRT and RRT runs on this problem
            data.write("\"DIRT\"," + DIRTruntime + "," + DIRTquality + "\n");
            data.write("\"RRT\"," + RRTruntime + "," + RRTquality + "\n");
            data.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Write the ratios to a csv file.
     */
    private void writeRatios() {
        try {
            boolean justCreated = false;
            File test = new File("ratios.csv");

            if (!test.exists()) {
                justCreated = true;
            }
            
            FileWriter data = new FileWriter("ratios.csv", true);

            // If this file was just created, add the first line for the value names
            if (justCreated) {
                data.write("\"Distribution\",\"DIRT:RRT Runtime Ratio\",\"DIRT:RRT Solution Cost Ratio\"\n");
            }

            // Append values for DIRT and RRT runs on this problem
            data.write(" ," + DIRTruntime / RRTruntime + "," + DIRTquality / RRTquality + "\n");
            data.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Write the ratios to a csv file.
     */
    private void writeDataForScatter() {
        try {
            boolean justCreated = false;
            File test = new File("scatter.csv");

            if (!test.exists()) {
                justCreated = true;
            }
            
            FileWriter data = new FileWriter("scatter.csv", true);

            // If this file was just created, add the first line for the value names
            if (justCreated) {
                data.write("\"DIRT Runtime\",\"RRT Runtime\",\"DIRT Solution Cost\",\"RRT Solution Cost\"\n");
            }

            // Append values for DIRT and RRT runs on this problem
            data.write(DIRTruntime + "," + RRTruntime + "," + DIRTquality + "," + RRTquality + "\n");
            data.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Parse a file containing a PTSP 
     * @param filename The file to parse
     */
    private void parseFile(String filename) {
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
                    coords = new double[2][2];
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
                }
            }

            scnr.close();
        } catch (FileNotFoundException e) {
            System.out.println("File not found.");
        }
    }
}