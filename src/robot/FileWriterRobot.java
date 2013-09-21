/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package robot;

import java.io.BufferedOutputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintStream;

/**
 *
 * @author drchaj1
 */
public class FileWriterRobot implements IRobotInterface { 
    PrintStream stream;

    public FileWriterRobot(PrintStream stream) {
        this.stream = stream;
    }

    public FileWriterRobot(String fileName) {
        try {
            stream = new PrintStream(new BufferedOutputStream(new FileOutputStream(fileName)));
        } catch (FileNotFoundException e) {
            e.printStackTrace();
            System.exit(1);
        }
    }

    public void setWheelSpeed(double left, double right) {
        stream.println(left + " " + right);
    }

    public double[] getSensorData() {
        throw new IllegalStateException("FileWriterRobot has no sensors!");
    }

    public void close() {
        stream.close();
    }
}