/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package robot.joystick;

import java.util.EventObject;

/**
 *
 * @author drchaj1
 */
public class JoystickEvent extends EventObject {
    private float x;
    private float y;

    public JoystickEvent(Object source, float x, float y) {
        super(source);
        this.x = x;
        this.y = y;
    }

    public float getX() {
        return x;
    }

    public float getY() {
        return y;
    }
}
