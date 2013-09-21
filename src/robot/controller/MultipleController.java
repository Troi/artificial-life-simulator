package robot.controller;

import robot.IRobotInterface;

/**
 * Created by IntelliJ IDEA.
 * User: drchaj1
 * Date: Apr 28, 2010
 * Time: 11:08:41 AM
 * To change this template use File | Settings | File Templates.
 */
public class MultipleController implements IRobotController {
    final private IRobotController[] controllers;

    public MultipleController(IRobotController[] controllers) {
        this.controllers = controllers.clone();
    }

    public void step() {
        for (IRobotController controller : controllers) {
            controller.step();
        }
    }

    public IRobotInterface getRobot() {
        return null;
    }
}
