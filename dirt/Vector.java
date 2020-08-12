package dirt;

/**
 * This class is used by the vehicle for storing the values of it's states (direction, velocity, and position). There
 * are also multiple methods included that can do simple math with the vectors to make the other code more streamlined.
 *
 * @author Lucas Guerrette
 * @version Ready for Version 0
 */
public class Vector {
    // Vector Fields
    public double x; // First (Top) number of the vector
    public double y; // Second (Bottom) number of the vector

    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * This method takes a single number and multiplies it to both of the vector values.
     * @param factor The number to multiply each value by
     */
    public void multOne(double factor) {
        x *= factor;
        y *= factor;
    }

    /**
     * This method takes two numbers and multiplies each vector value by each number.
     * @param fac1 The first number to be multiplied
     * @param fac2 The second number to be multiplied
     */
    public void multTwo(double fac1, double fac2) {
        x = x * fac1 * fac2;
        y = y * fac1 * fac2;
    }

    /**
     * This method is really only used in calculating the direction values, as it is essentially multiplying the
     * direction vector by a 2 x 2 vector of values, which come from the four parameters.
     * @param fac1 The top left value
     * @param fac2 The top right value
     * @param fac3 The bottom left value
     * @param fac4 The bottom right value
     */
    public void multFour(double fac1, double fac2, double fac3, double fac4) {
        double ogX = x;
        double ogY = y;

        x = (fac1 * ogX) + (fac3 * ogY);
        y = (fac2 * ogX) + (fac4 * ogY);
    }

    /**
     * This method adds a number to each of the vector values. (It is apparently unused)
     * @param num The number to add to each of the vector values
     */
    public void add(double num) {
        x += num;
        y += num;
    }

    /**
     * This method adds the values from one vector to the values of another.
     * @param v The vector whose values you want to add to the current vector
     */
    public void addVector(Vector v) {
        x += v.x;
        y += v.y;
    }

    /**
     * A method to create a copy of a vector. Useful in the calculations of the velocity of the vehicle.
     * @return A copy of the vector that this method was called on
     */
    public Vector copyVector() {
        return new Vector(x, y);
    }

    @Override
    public String toString() {
        return "<" + x + ", " + y + ">";
    }
}
