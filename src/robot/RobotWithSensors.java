package robot;

import vivae.blovstom.TomArena;
import vivae.arena.parts.Robot;

import java.awt.*;

/**
 * Created by IntelliJ IDEA.
 * User: drchaj1
 * Date: 1/12/11
 * Time: 12:43 PM
 * To change this template use File | Settings | File Templates.
 */
abstract public class RobotWithSensors extends Robot {
    public RobotWithSensors(float x, float y) {
        super(x, y);
    }

    public RobotWithSensors(Shape shape, int layer, TomArena arena) {
        super(shape, layer, arena);
    }

    public RobotWithSensors(float x, float y, TomArena arena) {
        super(x, y, arena);
    }

    abstract public double[] getSensoryData();
}
