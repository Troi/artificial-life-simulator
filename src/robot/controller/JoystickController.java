package robot.controller;

import robot.IRobotInterface;
import robot.joystick.JoystickDriver;
import robot.joystick.JoystickEvent;
import robot.joystick.JoystickListener;

/**
 * Created by IntelliJ IDEA.
 * User: drchaj1
 * Date: Apr 28, 2010
 * Time: 1:47:20 PM
 * To change this template use File | Settings | File Templates.
 */
public class JoystickController implements IRobotController {
    final private IRobotInterface robot;
    private final JoystickDriver driver;

    private double left;
    private double right;

    public JoystickController(final IRobotInterface robot) {
        this.robot = robot;
        driver = new JoystickDriver();
        driver.addJoystickListener(new JoystickListener() {
            public void joystickMoved(JoystickEvent e) {
                float posX = e.getX();
                float posY = e.getY();
                System.out.print("posX = " + posX + " posY = " + posY);

                double speed = -(double) posY;
                double ratio = 1 - Math.abs((double) posX);

                System.out.println(" speed = " + speed + " ratio = " + ratio);
                left = (posX < 0) ? (speed * ratio) : speed;
                right = (posX > 0) ? (speed * ratio) : speed;
            }
        });
    }

    public void step() {
        robot.setWheelSpeed(50 * left, 50 * right);
    }

    public IRobotInterface getRobot() {
        return robot;
    }

    public void stop() {
        driver.interruptPolling();
    }
}
