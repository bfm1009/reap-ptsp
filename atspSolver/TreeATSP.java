import java.io.File;
import java.io.FileWriter;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Scanner;

/**
 * An algorithm that solves small ATSPs by using a simple tree search.
 * @author Bryan McKenney
 * @version 3.0
 */
public class TreeATSP {
    private double[][] distances;
    private Tour bestTour;
    private long nodeCount;

    /**
     * Constructor.
     * @param fileName File that contains distance matrix
     */
    public TreeATSP(String fileName) {
        parseFileForMatrix(fileName);
        bestTour = null;
        int[] remainingNodes = new int[distances.length - 1];

        // Create the beginning of a tour starting at node 0
        Tour currTour = new Tour(0);

        // Put all node numbers except 0 in the remaining nodes list
        for (int i = 1; i <= remainingNodes.length; i++) {
            remainingNodes[i - 1] = i;
        }

        // Solve the ATSP (set the value of bestTour to be the shortest tour)
        solve(currTour, remainingNodes, remainingNodes.length, "");
    }

    /**
     * Parse a file containing a TSP or ATSP and set distances to the matrix representing that problem.
     * @param fileName The file to parse
     */
    private void parseFileForMatrix(String fileName) {
        distances = new double[][]{};

        try {
            File file = new File(fileName);
            Scanner scnr = new Scanner(file);

            // Initialize distance matrix and call function to finish parse depending on data format
            while (scnr.hasNext()) {
                String token = scnr.next();

                // Set the dimensions of the matrix
                if (token.equals("DIMENSION:")) {
                    int d = scnr.nextInt();
                    distances = new double[d][d];
                }

                // Start reading in values from matrix format
                else if (token.equals("EDGE_WEIGHT_SECTION")) {
                    parseMatrix(scnr);
                    break;
                }

                // Start reading in values from coordinate format
                else if (token.equals("NODE_COORD_SECTION")) {
                    parseCoords(scnr);
                    break;
                }
            }

            scnr.close();
        } catch (FileNotFoundException e) {
            System.out.println("File not found.");
        }
    }

    /**
     * Use file data containing a matrix to set the values in the distance matrix.
     * @param scnr Scanner containing file data
     */
    private void parseMatrix(Scanner scnr) {
        int row = 0;
        int col = 0;

        // Fill in distance matrix
        while (scnr.hasNextDouble()) {
            distances[row][col] = scnr.nextDouble();
            col++;

            // If done filling column, go to next row
            if (col >= distances.length) {
                col = 0;
                row++;
            }
        }
    }

    /**
     * Use file data containing 2D coordinates to set the values in the distance matrix.
     * @param scnr Scanner containing file data
     */
    private void parseCoords(Scanner scnr) {
        double[][] coords = new double[distances.length][2];
        int i = 0;

        // Read the coordinates into coords
        while (scnr.hasNextDouble()) {
            scnr.nextDouble(); // Skip number indicating node number
            coords[i][0] = scnr.nextDouble();
            coords[i][1] = scnr.nextDouble();
            i++;
        }

        // Calculate the distances between the coordinates to fill distances
        for (int row = 0; row < coords.length; row++) {
            for (int col = 0; col < coords.length; col++) {
                if (row != col) {
                    // Use the distance formula to find distance between the two nodes
                    double dis = Math.sqrt(Math.pow(coords[row][0] - coords[col][0], 2)
                            + Math.pow(coords[row][1] - coords[col][1], 2));

                    // Put that distance in both corresponding places in the matrix
                    distances[row][col] = dis;
                    distances[col][row] = dis;
                } else {
                    distances[row][col] = 0; // Set a node's distance to itself equal to 0
                }
            }
        }
    }

    /**
     * Solve the ATSP using a recursive tree search.
     * @param currTour The sequence of nodes visited so far and this partial tour's distance
     * @param remainingNodes The nodes left to visit
     */
    private void solve(Tour currTour, int[] remainingNodes, int remSize, String indent) {
        // Increment nodeCount
        nodeCount++;

        if (remSize == 1) {
            // Finish tour by adding last node and returning back to first node
            currTour.addNode(remainingNodes[0]);
            currTour.addNode(currTour.nodeOrder[0]);

            // If this tour is shorter than the best tour, update the best tour
            if (bestTour == null || currTour.distance < bestTour.distance) {
                bestTour = new Tour(currTour);
            }

            // Undo changes to currTour
            currTour.removeLastNode();
            currTour.removeLastNode();
        } else {
            for (int i = 0; i < remSize; i++) {
                // Sort the remaining nodes by distance to increase the chance that shorter tours are found first
                sortRemaining(currTour.getLastNode(), remainingNodes, remSize);
                //System.out.println(indent + "begin " + Arrays.toString(remainingNodes) + " i = " + i);

                // Remove node from remaining nodes and add it to to current tour
                int node = remainingNodes[i];
                remainingNodes[i] = remainingNodes[remSize - 1]; // Move the last element to this position
                //remainingNodes[remSize - 1] = 0; // Not necessary but helpful when debugging
                currTour.addNode(node);

                // If this partial tour is still shorter than the best tour, keep going
                if (bestTour == null || currTour.distance < bestTour.distance) {
                    solve(currTour, remainingNodes, remSize - 1, indent + "   ");
                }

                // Undo changes to currTour and remainingNodes
                currTour.removeLastNode();
                remainingNodes[remSize - 1] = remainingNodes[i];
                remainingNodes[i] = node;

                //System.out.println(indent + "end   " + Arrays.toString(remainingNodes));
            }
        }
    }

    /**
     * Sort the remainingNodes array in ascending order by how far each remaining node is from the last node.
     * Uses the insertion sort method.
     * @param lastNode The last node added to currTour
     * @param remainingNodes The array to be sorted
     * @param remSize How many valid elements are in remainingNodes
     */
    private void sortRemaining(int lastNode, int[] remainingNodes, int remSize) {
        for (int i = 1; i < remSize; i++) {
            int curr = remainingNodes[i];
            int j = i - 1;

            while (j > -1 && distances[lastNode][remainingNodes[j]] > distances[lastNode][curr]) {
                remainingNodes[j + 1] = remainingNodes[j];
                j--;
            }

            remainingNodes[j + 1] = curr;
        }
    }

    /**
     * Print the solution to the ATSP.
     */
    public void printSolution() {
        if (bestTour != null) {
            System.out.println("Best tour: " + bestTour);
            System.out.println("Distance: " + bestTour.distance);
        } else {
            System.out.println("No tour found.");
        }

        System.out.println("Node count: " + nodeCount);
    }

    /**
     * Write the solution to the ATSP to the specified file.
     * @param filename The file to create or override.
     */
    public void writeSolution(String filename) {
        if (bestTour != null) {
            try {
                FileWriter writer = new FileWriter(filename);

                // Write problem to file
                writer.write("SIZE: " + distances.length + "\n\n");
                writer.write("PROBLEM\n");

                for (int row = 0; row < distances.length; row++) {
                    for (int col = 0; col < distances.length; col++) {
                        writer.write(Double.toString(distances[row][col]));

                        // Add a space after all but last element
                        if (col != distances.length - 1) {
                            writer.write(" ");
                        }
                    }

                    // Newline after each row
                    writer.write("\n");
                }

                // Write solution to file
                writer.write("\nSOLUTION\n");

                for (int i = 0; i < bestTour.nodeOrder.length; i++) {
                    writer.write("" + bestTour.nodeOrder[i]);

                    // Add a space after all but last element
                    if (i != bestTour.nodeOrder.length - 1) {
                        writer.write(" ");
                    }
                }

                writer.close();
                System.out.println("Successfully wrote to file.");
            } catch (IOException e) {
                System.out.println("IO Exception.");
            }
        } else {
            System.out.println("Did not write to file.");
        }
    }

    /**
     * Checks that the solution is a valid tour and that its distance value is correct.
     * @return Whether the solution is valid or not
     */
    public boolean solutionIsValid() {
        double dis = 0;
        int[] tour = bestTour.nodeOrder;
        ArrayList<Integer> allNodes = new ArrayList<Integer>();

        // If tour is not the right size, return false
        if (tour.length != distances.length + 1) {
            System.out.println("Solution is invalid -- tour visits incorrect number of nodes.");
            return false;
        }

        // If tour doesn't loop back on itself, return false
        if (tour[0] != tour[tour.length - 1]) {
            System.out.println("Solution is invalid -- tour is not a loop.");
            return false;
        }

        // Fill a list with all the nodes that should be in the tour
        for (int i = 0; i < distances.length; i++) {
            allNodes.add(i);
        }

        // Make sure that every node is in the tour and that there are no duplicates (aside from the last)
        for (int i = 0; i < tour.length - 1; i++) {
            boolean found = false;

            for (int j = 0; j < allNodes.size(); j++) {
                // If the node in the tour is in allNodes, remove it from there and set found equal to true
                if (tour[i] == allNodes.get(j)) {
                    allNodes.remove(j);
                    found = true;
                    break;
                }
            }

            // If this node is not in allNodes, it is either an invalid node number or a duplicate, so return false
            if (!found) {
                System.out.println("Solution is invalid -- tour contains duplicates or invalid nodes.");
                return false;
            }
        }

        // Calculate distance of tour
        for (int i = 1; i < tour.length; i++) {
            dis += distances[tour[i - 1]][tour[i]];
        }

        // If bestTour's distance doesn't match actual distance (within margin of floating-point error), return false
        if (Math.abs(dis - bestTour.distance) > 0.0001) {
            System.out.println("Solution is invalid -- distance is inaccurate.");
            return false;
        }

        // If all the tests were passed, return true
        System.out.println("Solution is valid.");
        return true;
    }

    /**
     * Inner Tour class.
     */
    private class Tour {
        private int[] nodeOrder;
        private double distance;
        private int index;

        /**
         * Constructor.
         * @param firstNode The first node in the tour
         */
        public Tour(int firstNode) {
            nodeOrder = new int[distances.length + 1]; // +1 because tour must return to the starting node
            nodeOrder[0] = firstNode;
            distance = 0;
            index = 1;
        }

        /**
         * Copy constructor.
         * @param t The Tour object to make this one a copy of.
         */
        public Tour(Tour t) {
            nodeOrder = new int[t.nodeOrder.length];
            distance = t.distance;
            index = t.index;

            for (int i = 0; i < nodeOrder.length; i++) {
                nodeOrder[i] = t.nodeOrder[i];
            }
        }

        /**
         * Add a node to the tour.
         * @param node The node to add
         */
        public void addNode(int node) {
            if (index < nodeOrder.length) {
                // Add node at proper index
                nodeOrder[index] = node;

                // Add to distance (distance from last node to this one)
                distance += distances[nodeOrder[index - 1]][nodeOrder[index]];

                // Increment index
                index++;
            }
        }

        /**
         * Remove the last node in the tour.
         */
        public void removeLastNode() {
            if (index > 1) {
                // Decrease distance
                distance -= distances[nodeOrder[index - 2]][nodeOrder[index - 1]];

                // Reset element and decrement index
                nodeOrder[index - 1] = 0;
                index--;
            }
        }

        /**
         * Return the last node in the tour.
         * @return The last node
         */
        public int getLastNode() {
           return nodeOrder[index - 1];
        }

        @Override
        public String toString() {
            String str = "[";

            for (int i = 0; i < nodeOrder.length; i++) {
                str += nodeOrder[i];

                // Add a comma and space after element, unless it is the last
                if (i != nodeOrder.length - 1) {
                    str += ", ";
                }
            }

            str += "]";
            return str;
        }
    }

    /**
     * Run the program.
     * @param args Optional String[]
     */
    public static void main(String[] args) {
        // Get name of filename from user
        Scanner scnr = new Scanner(System.in);
        System.out.print("Enter filename: ");
        String filename = scnr.nextLine();

        // Solve an ATSP from a file
        TreeATSP atsp = new TreeATSP(filename);

        // Print the solution to the console
        atsp.printSolution();

        // If solution is valid, write it to a file
        if (atsp.solutionIsValid()) {
            atsp.writeSolution("solution.txt");
        } else {
            System.out.println("File unmodified.");
        }
    }
}