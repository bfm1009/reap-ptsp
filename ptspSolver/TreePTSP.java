package ptspSolver;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;

/**
 * An algorithm that solves small PTSPs by using a simple tree search.
 * @author Bryan McKenney
 * @version 6.0
 */
public class TreePTSP {
    private double[][] coords;
    private double[][] times;
    private Tour bestTour;
    private long nodeCount;
    private long nodeLimit;
    private boolean done;

    /**
     * Constructor.
     * @param coords Coordinates for the nodes
     * @param times The time matrix
     * @param nodeLimit How many nodes should be visited before execution stops
     */
    public TreePTSP(double[][] coords, double[][] times, long nodeLimit) {
        // Initialize field variables
        this.coords = coords;
        this.times = times;
        bestTour = null;
        nodeCount = 0;
        this.nodeLimit = nodeLimit;
        done = false;

        // Create an array for all the nodes to be visited
        int[] remainingNodes = new int[times.length - 1];

        // Put all node numbers except 0 in the remaining nodes array
        for (int i = 1; i <= remainingNodes.length; i++) {
            remainingNodes[i - 1] = i;
        }

        // Create the beginning of a tour starting at node 0
        Tour currTour = new Tour(0);

        // Solve the PTSP (set the value of bestTour to be the shortest tour)
        solve(currTour, remainingNodes, remainingNodes.length, "");
    }

    /**
     * Solve the PTSP using a recursive tree search.
     * @param currTour The sequence of nodes visited so far and this partial tour's time
     * @param remainingNodes The nodes left to visit
     */
    private void solve(Tour currTour, int[] remainingNodes, int remSize, String indent) {
        // Stop the program after a certain number of nodes have been looked at (a limit of -1 means never stop early)
        if (nodeCount == nodeLimit && nodeLimit != -1) {
            done = true;
            return;
        }

        // Increment nodeCount
        nodeCount++;

        if (remSize == 1) {
            // Finish tour by adding last node
            currTour.addNode(remainingNodes[0]);

            // If this tour is shorter than the best tour, update the best tour
            if (bestTour == null || currTour.time < bestTour.time) {
                bestTour = new Tour(currTour);
            }

            // Undo changes to currTour
            currTour.removeLastNode();
        } else {
            for (int i = 0; i < remSize && !done; i++) {
                // Sort the remaining nodes by time to increase the chance that shorter tours are found first
                sortRemaining(currTour.getLastNode(), remainingNodes, remSize);
                //System.out.println(indent + "begin " + Arrays.toString(remainingNodes) + " i = " + i);

                // Remove node from remaining nodes and add it to to current tour
                int node = remainingNodes[i];
                remainingNodes[i] = remainingNodes[remSize - 1]; // Move the last element to this position
                //remainingNodes[remSize - 1] = 0; // Not necessary but helpful when debugging
                currTour.addNode(node);

                // If this partial tour is still shorter than the best tour, keep going
                if (bestTour == null || currTour.time < bestTour.time) {
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

            while (j > -1 && times[lastNode][remainingNodes[j]] > times[lastNode][curr]) {
                remainingNodes[j + 1] = remainingNodes[j];
                j--;
            }

            remainingNodes[j + 1] = curr;
        }
    }

    /**
     * Returns the best node ordering.
     * @return An array of ordered node numbers
     */
    public int[] getOrdering() {
        if (bestTour != null) {
            return bestTour.nodeOrder;
        }

        return null;
    }

    /**
     * Returns a 2D array of doubles. The first element is the initial position of the vehicle,
     * and the other elements are the coordinates of the ordered waypoints.
     * @return The solution
     */
    public double[][] getSolution() {
        double[][] solution = null;

        if (bestTour != null) {
            int[] tour = bestTour.nodeOrder;

            // Initialize solution array
            solution = new double[times.length][2];

            // Fill in the array by looping through the best tour and finding the coordinates for each node
            for (int i = 0; i < tour.length; i++) {
                solution[i] = coords[tour[i]];
            }
        }

        return solution;
    }

    /**
     * Print the solution to the PTSP.
     */
    public void printSolution() {
        if (bestTour != null) {
            System.out.println("Best tour: " + bestTour);
            System.out.println("Time: " + bestTour.time);
        } else {
            System.out.println("No tour found.");
        }

        System.out.println("Node count: " + nodeCount);
    }

    /**
     * Write the solution to the PTSP to the specified file.
     * @param filename The file to create or override.
     */
    public void writeSolution(String filename) {
        if (bestTour != null) {
            try {
                FileWriter writer = new FileWriter(filename);

                // Write problem to file
                writer.write("SIZE: " + times.length + "\n\n");
                writer.write("PROBLEM\n");

                for (int row = 0; row < times.length; row++) {
                    for (int col = 0; col < times.length; col++) {
                        writer.write(Double.toString(times[row][col]));

                        // Add a space after all but last element
                        if (col != times.length - 1) {
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
     * Checks that the solution is a valid tour and that its time value is correct.
     * @return Whether the solution is valid or not
     */
    public boolean solutionIsValid() {
        int[] tour;
        double t = 0;
        ArrayList<Integer> allNodes = new ArrayList<Integer>();

        // If a tour was never found, return false
        if (bestTour != null) {
            tour = bestTour.nodeOrder;
        } else {
            System.out.println("No solution to validate.");
            return false;
        }

        // If tour is not the right size, return false
        if (tour.length != times.length) {
            System.out.println("Solution is invalid -- tour visits incorrect number of nodes.");
            return false;
        }

        // Fill a list with all the nodes that should be in the tour
        for (int i = 0; i < times.length; i++) {
            allNodes.add(i);
        }

        // Make sure that every node is in the tour and that there are no duplicates
        for (int i = 0; i < tour.length; i++) {
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

        // Calculate time of tour
        for (int i = 1; i < tour.length; i++) {
            t += times[tour[i - 1]][tour[i]];
        }

        // If bestTour's time doesn't match actual time (within margin of floating-point error), return false
        if (Math.abs(t - bestTour.time) > 0.0001) {
            System.out.println("Solution is invalid -- time is inaccurate.");
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
        private double time;
        private int index;

        /**
         * Constructor.
         * @param firstNode The first node in the tour
         */
        public Tour(int firstNode) {
            nodeOrder = new int[times.length];
            nodeOrder[0] = firstNode;
            time = 0;
            index = 1;
        }

        /**
         * Copy constructor.
         * @param t The Tour object to make this one a copy of.
         */
        public Tour(Tour t) {
            nodeOrder = new int[t.nodeOrder.length];
            time = t.time;
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

                // Add to time (time from last node to this one)
                time += times[nodeOrder[index - 1]][nodeOrder[index]];

                // Increment index
                index++;
            }
        }

        /**
         * Remove the last node in the tour.
         */
        public void removeLastNode() {
            if (index > 1) {
                // Decrease time
                time -= times[nodeOrder[index - 2]][nodeOrder[index - 1]];

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
}