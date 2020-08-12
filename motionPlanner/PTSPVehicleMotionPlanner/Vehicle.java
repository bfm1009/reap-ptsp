import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

public class Vehicle {
    //Fields of the vehicle ----------------------------------------------------------------------
    double lAlpha = -Math.PI / 60;
    double rAlpha = Math.PI / 60;
    double friction = 0.99;
    double accConstant = 0.025;
    double timeStep = 0.05;
    Vector direction;
    Vector velocity;
    Vector position;

    public Vehicle(Vector dir, Vector vel, Vector pos) {
        direction = dir;
        velocity = vel;
        position = pos;
    }

    public void updateDirectionVec(Vector direction, int turn) {
        if (turn == 1) {
            direction.multFour(Math.cos(rAlpha), -Math.sin(rAlpha), Math.sin(rAlpha), Math.cos(rAlpha));
        } else if (turn == -1) {
            direction.multFour(Math.cos(lAlpha), -Math.sin(lAlpha), Math.sin(lAlpha), Math.cos(lAlpha));
        } else {
            direction.multFour(Math.cos(0), -Math.sin(0), Math.sin(0), Math.cos(0));
        }
    }

    public void updateVelocityVec(Vector velocity, Vector direction, int accBoolean) {
        Vector dirCopy = direction.copyVector();
        dirCopy.multTwo(accBoolean, accConstant);
        velocity.addVector(dirCopy);
        velocity.multOne(friction);
    }

    public void updatePositionVec(Vector position, Vector velocity) {
        position.addVector(velocity);
    }

    public ArrayList<double[]> test(Vector dir, Vector vel, Vector pos) {
        ArrayList<double[]> states = new ArrayList<>();
        for (double i = 0; i <= 16; i += timeStep) {
            double[] state = new double[3];
            if (i < 1.5) {
                updateDirectionVec(dir, -1);
                state[2] = Math.atan2(dir.y, dir.x);
            } else if (i < 3) {
                updateDirectionVec(dir, 1);
                state[2] = Math.atan2(dir.y, dir.x);
            } else if (i < 6) {
                updateDirectionVec(dir, 0);
                state[2] = Math.atan2(dir.y, dir.x);
            } else if (i < 8.25) {
                updateDirectionVec(dir, 1);
                state[2] = Math.atan2(dir.y, dir.x);
            } else {
                updateDirectionVec(dir, 0);
                state[2] = Math.atan2(dir.y, dir.x);
            }
            updateVelocityVec(vel, dir, 1);
            updatePositionVec(pos, vel);
            state[0] = pos.x;
            state[1] = pos.y;
            states.add(state);
        }
        return states;
    }

    public static void main(String[] args) {
        Vector dir = new Vector(1, 0);
        Vector vel = new Vector(0, 0);
        Vector pos = new Vector(0, 0);

        Vehicle v = new Vehicle(dir, vel, pos);

        for (int i = 0; i < 3; i++) {
            v.updateDirectionVec(v.direction, 1);
            v.updateVelocityVec(v.velocity, v.direction, 1);
            v.updatePositionVec(v.position, v.velocity);
        }

        System.out.println("Direction: " + v.direction.x + ", " + v.direction.y);
        System.out.println("Velocity: " + v.velocity.x + ", " + v.velocity.y);
        System.out.println("Position: " + v.position.x + ", " + v.position.y);

        /*ArrayList<double[]> states = v.test(dir, vel, pos);
        try {
            FileWriter fw = new FileWriter("testpoints.txt", false);
            System.out.println(states.size());
            fw.write(states.size() + "\n");
            for (int i = 0; i < states.size(); i++) {
                System.out.println(states.get(i)[0] + " " + states.get(i)[1] + " " + states.get(i)[2]);
                fw.write(states.get(i)[0] + " " + states.get(i)[1] + " " + states.get(i)[2] + "\n");
            }
            fw.close();
        } catch (IOException e) {
            e.printStackTrace();
        }*/
    }
}
