package necromancer;

public class Headings {
    // Fields ------------------------------------------------------
    double[][] input; // The 2d array coming from the PTSP Waypoint Solver
    double[] startState = new double[6]; // Has 6 spots for each value of the three necessary vectors
    // -------------------------------------------------------------

    public Headings(double[] initDir, double[][] solverOutput) {
        input = solverOutput;
        startState[0] = initDir[0]; // X value for direction vector
        startState[1] = initDir[1]; // Y value for direction vector
        startState[2] = 0; // X velocity (starts at 0)
        startState[3] = 0; // Y velocity (starts at 0)
        startState[4] = input[0][0]; // X position
        startState[5] = input[0][1]; // Y position
    }


    public double[][] calcEnd() {
        double[][] states = new double[input.length][];
        double[] endState;
        int index = 1;
        states[0] = startState;
        for (int i = 1; i < input.length; i++) {
            double angle = Math.atan2(input[i][1] - startState[5], input[i][0] - startState[4]);
            double dirX = Math.cos(angle); // Calculates X direction
            double dirY = Math.sin(angle); // Calculates Y direction
            double xPos = input[i][0]; // Calls X position of target waypoint
            double yPos = input[i][1]; // Calls Y position of target waypoint
            endState = new double[] {dirX, dirY, 0, 0, xPos, yPos};
            startState = new double[]{dirX, dirY, 0, 0, xPos, yPos};
            states[index] = endState;
            index++;
        }
        for (double[] value : states) {
            for (double dub : value) {
                System.out.print(dub + " ");
            }
            System.out.println();
        }
        return states;
    }

    /*public static void main(String[] args) {
        Headings h = new Headings(new double[][] {{0.5, 0.5}, {1, 1}, {5, 3}, {6, 10}, {5, 13}});
        h.calcEnd();
    }*/
}
