package robot.controller;

import robot.IRobotInterface;

/**
 * Created by IntelliJ IDEA.
 * User: drchaj1
 * Date: Apr 27, 2010
 * Time: 9:22:01 PM
 * To change this template use File | Settings | File Templates.
 */
public class DemoController implements IRobotController {
    final private IRobotInterface robot;
    private int stepCount = 0;

    public DemoController(IRobotInterface robot) {
        this.robot = robot;
    }

    public void step() {
        if (stepCount < 50) {
            robot.setWheelSpeed(50, 50);
        } else if (stepCount < 100) {
            robot.setWheelSpeed(-50, -50);
        }
        stepCount++;
    }

    public IRobotInterface getRobot() {
        return robot;
    }
}
