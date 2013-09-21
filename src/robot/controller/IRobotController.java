package robot.controller;

import robot.IRobotInterface;

/**
 * Created by IntelliJ IDEA.
 * User: drchaj1
 * Date: Apr 27, 2010
 * Time: 9:19:50 PM
 * To change this template use File | Settings | File Templates.
 */
public interface IRobotController {
    void step();

    IRobotInterface getRobot();
}
