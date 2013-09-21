package robot.controller;

import robot.IRobotInterface;

/**
 * Created by IntelliJ IDEA.
 * User: drchaj1
 * Date: Apr 27, 2010
 * Time: 11:08:26 PM
 * To change this template use File | Settings | File Templates.
 */
public class UIUserController implements IRobotController {
    final private IRobotInterface robot;

    public UIUserController(IRobotInterface robot) {
        this.robot = robot;
    }

    public void step() {
        //no code needed here, the robot is driven by user
    }

    public IRobotInterface getRobot() {
        return robot;
    }
}
