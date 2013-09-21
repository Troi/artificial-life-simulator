/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package robot;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import robot.eye.EyeImage;

/**
 *
 * @author drchaj1
 */
public class HardwareRobotAdapter implements IHardwareRobotInterface {
    final private IRobotInterface adaptedRobot;

    public HardwareRobotAdapter(IRobotInterface adaptedRobot) {
        this.adaptedRobot = adaptedRobot;
    }

    public void connect() throws IOException {
    }

    public void disconnect() throws IOException {
    }

    public void setMaxWheelSpeed(double speed) throws IOException {
    }

    public EyeImage getEyeImage(String what, boolean capture) throws IOException {
        return null;
    }

    public Map<String, Object> getStatus() throws IOException {
        Map<String, Object> status = new HashMap<String, Object>();
        status.put("software_robot", 1);
        return status;
    }

    public String getHostname() {
        return "adapted_robot";
    }

    public int getPort() {
        return 0;
    }

    public void setWheelSpeed(double left, double right) {
        adaptedRobot.setWheelSpeed(left, right);
    }

    public double[] getSensorData() {
        throw new IllegalStateException("Not yet implemented!");
    }
}
