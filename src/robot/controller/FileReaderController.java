package robot.controller;

import robot.IRobotInterface;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Scanner;

/**
 * Created by IntelliJ IDEA.
 * User: drchaj1
 * Date: Apr 28, 2010
 * Time: 3:07:39 PM
 * To change this template use File | Settings | File Templates.
 */
public class FileReaderController implements IRobotController {
    final private IRobotInterface robot;

    private class LeftRight {
        double left;
        double right;

        private LeftRight(double left, double right) {
            this.left = left;
            this.right = right;
        }
    }

    List<LeftRight> path = new ArrayList<LeftRight>();
    private Iterator<LeftRight> iterator;

    public FileReaderController(IRobotInterface robot, String fileName) {
        this.robot = robot;
        try {
            Scanner scanner = new Scanner(new File(fileName));

            while(scanner.hasNextDouble()) {
                double left = scanner.nextDouble();
                double right = scanner.nextDouble();
                path.add(new LeftRight(left, right));
            }

            scanner.close();
            iterator = path.iterator();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
            System.exit(1);
        }
    }

    public void step() {
        if(iterator.hasNext()) {
            LeftRight leftRight = iterator.next();
//            System.out.println(leftRight.left + " " + leftRight.right);
            robot.setWheelSpeed(leftRight.left, leftRight.right);
        } else {
            
        }
    }

    public IRobotInterface getRobot() {
        return robot;
    }
}
