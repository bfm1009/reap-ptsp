import java.io.File;
import java.io.FileWriter;
import java.util.Scanner;

class Quotient {
    public static void main(String[] args) {
        String filename = args[0];
        int numSeeds = Integer.parseInt(args[1]);
        new Quotient(filename, numSeeds);
    }

    public Quotient(String filename, int numSeeds) {
        getAverages(filename, numSeeds);
    }

    public void getAverages(String filename, int numSeeds) {
        try {
            File file = new File(filename);
            Scanner scnr = new Scanner(file);
            int seedNum = 0;
            double bestRuntime = 10000;
            double bestCost = 10000;
            double[] DIRTruntimes = new double[numSeeds];
            double[] DIRTcosts = new double[numSeeds];
            double[] RRTruntimes = new double[numSeeds];
            double[] RRTcosts = new double[numSeeds];
            scnr.nextLine(); // Skip the first line (value names)

            while (scnr.hasNext()) {
                String line = scnr.nextLine();
                String nextLine = scnr.nextLine();
                String[] DIRTvalues = line.split(",");
                String[] RRTvalues = nextLine.split(",");
                double DIRTruntime = Double.parseDouble(DIRTvalues[1]);
                double DIRTcost = Double.parseDouble(DIRTvalues[2]);
                double RRTruntime = Double.parseDouble(RRTvalues[1]);
                double RRTcost = Double.parseDouble(RRTvalues[2]);

                // Find the best runtime
                if (DIRTruntime < bestRuntime) {
                    bestRuntime = DIRTruntime;
                } else if (RRTruntime < bestRuntime) {
                    bestRuntime = RRTruntime;
                }

                // Find the best cost
                if (DIRTcost < bestCost) {
                    bestCost = DIRTcost;
                } else if (RRTcost < bestCost) {
                    bestCost = RRTcost;
                }

                // Add values to arrays
                DIRTruntimes[seedNum] = DIRTruntime;
                DIRTcosts[seedNum] = DIRTcost;
                RRTruntimes[seedNum] = RRTruntime;
                RRTcosts[seedNum] = RRTcost;
                
                // Increment seedNum
                seedNum++;
            }
            
            //Initialize the total values of all the quotients to calculate the average from after the loop
            double DIRTRunQuoTot = 0;
            double DIRTCostQuoTot = 0;
            double RRTRunQuoTot = 0;
            double RRTCostQuoTot  = 0;

            for (int i = 0; i < numSeeds; i++) {
                DIRTRunQuoTot += DIRTruntimes[i] / bestRuntime;   
                DIRTCostQuoTot += DIRTcosts[i] / bestCost;
                RRTRunQuoTot += RRTruntimes[i] / bestRuntime;
                RRTCostQuoTot += RRTcosts[i] / bestCost;
            }

            DIRTRunQuoTot /= numSeeds;
            DIRTCostQuoTot /= numSeeds;
            RRTRunQuoTot /= numSeeds;
            RRTCostQuoTot /= numSeeds;

            scnr.close();

            // Append the averages of the quotients to a file ---------------------------------
            boolean justCreated = false;
            File test = new File("averages.csv");

            if (!test.exists()) {
                justCreated = true;
            }
            
            FileWriter data = new FileWriter("averages.csv", true);

            // If this file was just created, add the first line for the value names
            if (justCreated) {
                data.write("\"Motion Planning Algorithm\",\"Runtime\",\"Solution Cost\"\n");
            }

            // Append values for DIRT and RRT runs on this problem
            data.write("\"DIRT\"," + DIRTRunQuoTot + "," + DIRTCostQuoTot + "\n");
            data.write("\"RRT\"," + RRTRunQuoTot + "," + RRTCostQuoTot + "\n");
            data.close();
        } catch (Exception e) {
            System.out.println("The ugly code broke :,(");
            e.printStackTrace();
        }
    }
}