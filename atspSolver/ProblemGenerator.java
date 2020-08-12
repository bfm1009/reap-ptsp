import java.awt.geom.Point2D;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Scanner;

/**
 * Class that takes user input and generates a Euclidean .tsp file.
 * @author Bryan McKenney
 * @version 1.0
 */
public class ProblemGenerator {
    private ArrayList<Point2D> coords;
    private double optimalDis;

    /**
     * Constructor.
     */
    public ProblemGenerator() {
        coords = new ArrayList<Point2D>();
        optimalDis = 0;
        Scanner scnr = new Scanner(System.in);
        String filename;
        int nodeNum;
        double angleMod;
        double x, y;

        // Get desired filename from user
        System.out.print("Enter a filename to write to (without extension): ");
        filename = scnr.next();

        // Get number of nodes from user
        System.out.print("Enter the number of nodes: ");
        nodeNum = scnr.nextInt();

        // Calculate angle between each node (from center of a circle)
        angleMod = 360.0 / nodeNum;

        // Create points for unit-[nodeNum-sided polygon]
        for (int i = 0; i < nodeNum; i++) {
            x = Math.cos(Math.toRadians(angleMod * i));
            y = Math.sin(Math.toRadians(angleMod * i));
            Point2D pt = new Point2D.Double(x, y);
            coords.add(pt);

            // Add distance between this point and the last one to optimalDis
            if (i > 0) {
                optimalDis += Math.sqrt(Math.pow(x - coords.get(i - 1).getX(), 2)
                        + Math.pow(y - coords.get(i - 1).getY(), 2));
            }
        }

        // Finish optimalDis by adding distance between last and first coordinate
        optimalDis += Math.sqrt(Math.pow(coords.get(0).getX() - coords.get(nodeNum - 1).getX(), 2)
                + Math.pow(coords.get(0).getY() - coords.get(nodeNum - 1).getY(), 2));

        // Shuffle points so that they don't start in the best order
        Collections.shuffle(coords);

        // Write coords to file
        writeToFile(filename);
    }

    /**
     * Write the contents of coords to a file with the given name.
     * @param filename The name of the file to write to (without file extension)
     */
    private void writeToFile(String filename) {
        try {
            FileWriter writer = new FileWriter(filename + ".tsp");

            // Write info
            writer.write("NAME: " + filename + "\n");
            writer.write("TYPE: TSP\n");
            writer.write("COMMENT: Optimal distance is " + optimalDis + "\n");
            writer.write("DIMENSION: " + coords.size() + "\n");
            writer.write("EDGE_WEIGHT_TYPE: EUC_2D\n");
            writer.write("NODE_COORD_TYPE: TWOD_COORDS\n");
            writer.write("DISPLAY_DATA_TYPE: COORD_DISPLAY\n");
            writer.write("NODE_COORD_SECTION\n");

            // Write coordinates
            for (int i = 0; i < coords.size(); i++) {
                Point2D pt = coords.get(i);
                writer.write("" + (i + 1) + "\t" + pt.getX() + " " + pt.getY() + "\n");
            }

            // Write end of file
            writer.write("EOF");

            writer.close();
            System.out.println("Successfully wrote to file.");
        } catch (IOException e) {
            System.out.println("IO Exception.");
        }
    }

    /**
     * Run the program.
     * @param args Optional String[]
     */
    public static void main(String[] args) {
        // Run the problem generator
        ProblemGenerator gen = new ProblemGenerator();
    }
}
