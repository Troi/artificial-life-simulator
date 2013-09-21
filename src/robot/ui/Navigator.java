/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package robot.ui;

import robot.IRobotInterface;
import robot.joystick.JoystickDriver;
import robot.joystick.JoystickEvent;
import robot.joystick.JoystickListener;

import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;

/**
 * @author mirek
 */
public class Navigator extends JComponent implements MouseListener,
        MouseMotionListener, JoystickListener {

//    private final static double duration = 5.0; // 5 seconds
    private int posX = 0;
    private int posY = 0;
    private int touchX = 0;
    private int touchY = 0;
    private double leftWheelSpeedScale;
    private double rightWheelSpeedScale;
    private static final int graphBorder = 20;
    private IRobotInterface robot;
    private JoystickDriver joystickDriver;

    public Navigator() {
        super();
        addMouseListener(this);
        addMouseMotionListener(this);
        /*
        Timer timer = new Timer();
        timer.scheduleAtFixedRate(t, 0, 100);
        TimerTask t = new TimerTask() {

        @Override
        public void run() {

        }
        }
         */
    }

    public void setRobot(IRobotInterface robot) {
        this.robot = robot;
    }

    public void setJoystickController(JoystickDriver joystickDriver) {
        this.joystickDriver = joystickDriver;
        if (joystickDriver != null) {
            removeMouseListener(this);
            removeMouseMotionListener(this);
            joystickDriver.addJoystickListener(this);
        }
    }

    public IRobotInterface getRobot() {
        return robot;
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Rectangle r = getBounds();

        int gwidth = r.width - 2 * graphBorder;
        int gheight = r.height - 2 * graphBorder;

        Graphics gg = g.create();
        int offX = r.width / 2;
        int offY = r.height / 2;
        gg.setColor(Color.black);
        gg.drawLine(offX, offY - gheight / 2 - 10, offX, offY + gheight / 2 + 10);
        gg.drawLine(offX - gwidth / 2 - 10, offY, offX + gwidth / 2 + 10, offY);
        int tickX = (r.width - 25) / 10;
        int tickY = (r.height - 25) / 10;
        for (int i = 1; i <= 5; i++) {
            gg.drawLine(offX - tickX * i, offY + 5, offX - tickX * i, offY - 5);
            gg.drawLine(offX + tickX * i, offY + 5, offX + tickX * i, offY - 5);
            gg.drawLine(offX + 5, offY - tickY * i, offX - 5, offY - tickY * i);
            gg.drawLine(offX + 5, offY + tickY * i, offX - 5, offY + tickY * i);
        }
        gg.setColor(Color.red);
        gg.fillOval(posX + offX - 20, posY + offY - 20, 40, 40);
    }

    public void mouseClicked(MouseEvent e) {
    }

    public void mousePressed(MouseEvent e) {
        touchX = e.getX();
        touchY = e.getY();
        posX = 0;
        posY = 0;
        repaint();
    }

    public void mouseReleased(MouseEvent e) {
        posX = 0;
        posY = 0;
//        try {
        if (robot == null) {
            return;
        }
//            robot.setWheelSpeed(0, 0, duration);
        robot.setWheelSpeed(0, 0);
//        } catch (IOException ex) {
//            Logger.getLogger(Navigator.class.getName()).log(Level.SEVERE, null, ex);
//        }
        repaint();
    }

    public void mouseEntered(MouseEvent e) {
    }

    public void mouseExited(MouseEvent e) {
    }

    public void mouseDragged(MouseEvent e) {
        Rectangle r = getBounds();
        int gwidth = r.width - 2 * graphBorder;
        int gheight = r.height - 2 * graphBorder;
        posX = e.getX() - touchX;
        posX = Math.min(posX, gwidth / 2);
        posX = Math.max(posX, -gwidth / 2);
        posY = e.getY() - touchY;
        posY = Math.min(posY, gheight / 2);
        posY = Math.max(posY, -gheight / 2);

        robotMove(gwidth, gheight);

        repaint();
    }

    public void mouseMoved(MouseEvent e) {
    }

    public void joystickMoved(JoystickEvent e) {
        Rectangle r = getBounds();
        int gwidth = r.width - 2 * graphBorder;
        int gheight = r.height - 2 * graphBorder;
        posX = (int) (e.getX() * (gwidth / 2));
        posY = (int) (e.getY() * (gheight / 2));

        robotMove(gwidth, gheight);

        repaint();
    }

    private void robotMove(int gwidth, int gheight) {
        if (robot != null) {
//            try {
            double speed = -(double) posY / (double) gheight * 2.0 * 100.0;
            double ratio = 1 - Math.abs((double) posX) / (double) gwidth * 2;
            double leftSpeed = (posX < 0) ? (speed * ratio) : speed;
            double rightSpeed = (posX > 0) ? (speed * ratio) : speed;
            robot.setWheelSpeed(leftSpeed, rightSpeed);
//                robot.setWheelSpeed(leftSpeed, rightSpeed, duration);
//            } catch (IOException ex) {
//                Logger.getLogger(Navigator.class.getName()).log(Level.SEVERE, null, ex);
//            }
        }
    }
}
