/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package robot.joystick;

import javax.swing.event.EventListenerList;
import net.java.games.input.Component;
import net.java.games.input.Controller;
import net.java.games.input.ControllerEnvironment;
import net.java.games.input.Event;
import net.java.games.input.EventQueue;

/**
 *
 * @author drchaj1
 */
public class JoystickDriver {

    private static int AXES_COUNT = 2;
    Controller joystick;
    Component[] axes = new Component[AXES_COUNT];
    float[] lastState = new float[]{0.0f, 0.0f};
    private EventListenerList eventListenerList = new EventListenerList();
    private Thread joystickThread;

    private boolean running = false;

    public JoystickDriver() {
        findJoystick();
        selectAxes();
        joystickThread = new Thread() {

            @Override
            public void run() {
                pollEvents();
            }

        };
        startPolling();
    }

    public void startPolling() {
        joystickThread.start();
        running = true;
    }

    public void interruptPolling() {
        running = false;
    }


    private void findJoystick() {
        Controller[] controllerList =
                ControllerEnvironment.getDefaultEnvironment().getControllers();
        for (Controller controller : controllerList) {
            if (controller.getType() == Controller.Type.STICK) {
                joystick = controller;
                System.out.println("Found joystick: " + joystick.getName());
                return;
            }
        }
        throw new IllegalStateException("No joystick found!");
    }

    private void selectAxes() {
        axes[0] = getAxisByName("x");
//        axes[0] = getAxisByName("rz");
        axes[1] = getAxisByName("y");
        printComponentDetails(axes[0]);
        printComponentDetails(axes[1]);
    }

    private Component getAxisByName(String name) {
        for (Component component : joystick.getComponents()) {
            if (component.getIdentifier().getName().equals(name)) {
                return component;
            }
        }
        throw new IllegalStateException("Cannot find axis: \"" + name + "\"");
    }

    private void pollEvents() {
        EventQueue eventQueue = joystick.getEventQueue();
        Event event = new Event();
        while (running) {
            eventQueue.getNextEvent(event);
            // update all Axis instances of this controller
            joystick.poll();
            // update the text field's contents
            if (isActiveAxisAndChanged(event.getComponent())) {
                fireJoystickEvent(new JoystickEvent(this,
                        axes[0].getPollData(), axes[1].getPollData()));
//                System.out.println("joystick x: " + axes[0].getPollData() + " y:" + axes[1].getPollData());
            }
            try {
                Thread.sleep(10);
            } catch (InterruptedException ex) {
                ex.printStackTrace();
            }
        }
    }

    private boolean isActiveAxisAndChanged(Component c) {
        for (int i = 0; i < axes.length; i++) {
            if (axes[i] == c && lastState[i] != axes[i].getPollData()) {
                lastState[i] = axes[i].getPollData();
                return true;
            }
        }
        return false;
    }

    public void addJoystickListener(JoystickListener listener) {
        eventListenerList.add(JoystickListener.class, listener);
    }

    // This methods allows classes to unregister for MyEvents
    public void removeJoystickListener(JoystickListener listener) {
        eventListenerList.remove(JoystickListener.class, listener);
    }

    private void fireJoystickEvent(JoystickEvent e) {
        Object[] listeners = eventListenerList.getListenerList();
        // Each listener occupies two elements - the first is the listener class
        // and the second is the listener instance
        for (int i = 0; i < listeners.length; i += 2) {
            if (listeners[i] == JoystickListener.class) {
                ((JoystickListener) listeners[i + 1]).joystickMoved(e);
            }
        }
    }

    private static void printComponentDetails(Component component) {
        System.out.println(
                " - " + component.getName()
                + " - " + component.getIdentifier().getName()
                + " - " + (component.isRelative() ? "relative" : "absolute")
                + " - " + (component.isAnalog() ? "analog" : "digital")
                + " - " + component.getDeadZone());

    }
}
