package robot;

/**
 * Created by IntelliJ IDEA.
 * User: drchaj1
 * Date: Apr 27, 2010
 * Time: 9:12:49 PM
 * To change this template use File | Settings | File Templates.
 */
public class VivaeRobot implements IRobotInterface {
    private double left;
    private double right;
    private RobotWithSensors robot;

    public VivaeRobot(RobotWithSensors robot) {
        this.robot = robot;
    }

    public void setWheelSpeed(double left, double right) {
        this.left = left;
        this.right = right;
    }

    public double[] getSensorData() {
        return robot.getSensoryData();
    }

    public double getLeft() {
        return left;
    }

    public double getRight() {
        return right;
    }
}
