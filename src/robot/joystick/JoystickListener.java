/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package robot.joystick;

import java.util.EventListener;

/**
 *
 * @author drchaj1
 */
public interface JoystickListener extends EventListener {
    void joystickMoved(JoystickEvent e);
}
