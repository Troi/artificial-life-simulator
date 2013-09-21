package vivae.blovstom;

import java.util.Vector;
import vivae.arena.parts.Active;
import vivae.controllers.RobotWithSensorController;
import vivae.controllers.VivaeController;
import vivae.util.Util;


import vivae.controllers.nn.FRNN;

/**
 * Created by IntelliJ IDEA.
 * User: koutnij
 * Date: Nov 9, 2009
 * Time: 2:57:30 PM
 * To change this template use File | Settings | File Templates.
 */
public class TomController extends RobotWithSensorController {

    protected FRNN frnn = new FRNN();

    @Override
    public void moveControlledObject() {



        if (robot instanceof TomBot) {

            double[] input = Util.flatten(((TomBot) robot).getSensoryData());
            double[] eval = frnn.evalNetwork(input);

            double lWheel = eval[0];
            double rWheel = eval[eval.length - 1];
            double angle;
            double acceleration = 5.0 * (lWheel + rWheel);
            if (acceleration < 0) {
                acceleration = 0; // negative speed causes problems, why?
            }
            double speed = Math.abs(robot.getSpeed() / robot.getMaxSpeed());
            speed = Math.min(Math.max(speed, -1), 1);
            if (rWheel > lWheel) {
                angle = 10 * (1.0 - speed);
            } else {
                angle = -10 * (1.0 - speed);
            }

            //double mouth = eval[eval.length/2];

            robot.rotate((float) angle);
            robot.accelerate((float) acceleration);
            robot.eat();
        }

//        if (robot instanceof TomBot) {
//
//            double[] input = Util.flatten(((TomBot) robot).getSensoryData());
//            double[] eval = frnn.evalNetwork(input);
//
//            double lWheel = eval[0];
//            double rWheel = eval[eval.length - 1];
//            double angle;
//            double acceleration = 5.0 * (lWheel + rWheel);
//            if (acceleration < 0) {
//                acceleration = 0; // negative speed causes problems, why?
//            }
//            double speed = Math.abs(robot.getSpeed() / robot.getMaxSpeed());
//            speed = Math.min(Math.max(speed, -1), 1);
//            if (rWheel > lWheel) {
//                angle = 10 * (1.0 - speed);
//            } else {
//                angle = -10 * (1.0 - speed);
//            }
//
//            //double mouth = eval[eval.length/2];
//
//            robot.rotate((float) angle);
//            robot.accelerate((float) acceleration);
//            robot.eat();
//        }

    }

    public void setFRNN(FRNN net) {
        this.frnn = net;
    }

    public FRNN getFRNN() {
        return frnn;
    }

    static int neurons;
    static int sensors;
    static int min;
    static int max;

    static void setParameters(int neurons, int sensors, int min, int max) {
        TomController.neurons = neurons;
        TomController.sensors = sensors;
        TomController.min = min;
        TomController.max = max;
    }

    public void intitRandom() {

        double[][] wm = Util.randomArray2D(neurons, sensors + neurons + 1, -10, 10);

        initFRNN(Util.subMat(wm, 0, sensors - 1),
                Util.subMat(wm, sensors, sensors + neurons - 1),
                Util.flatten(Util.subMat(wm, sensors + neurons, sensors + neurons)));

    }

    public void intitOneParent(VivaeController controller) {
        double[][] wm = Util.randomArray2D(neurons, sensors + neurons + 1, -10, 10);

        FRNN mom = ((TomController) controller).getFRNN();

        int snum = (wm[0].length - neurons - 1);

        for (int i = 0; i < wm.length; i++) {
            for (int j = 0; j < snum; j++) {
                if (Math.random() < 0.9) {
                    wm[i][j] = mom.getwIn()[i][j];
                }
            }

            for (int j = snum; j < snum + neurons; j++) {
                if (Math.random() < 0.9) {
                    wm[i][j] = mom.getwRec()[i][j - snum];
                }

            }

            int j = snum + neurons;

            if (Math.random() < 0.9) {
                wm[i][j] = mom.getwThr()[i];
            }
        }

        wm = mutate(wm);

        initFRNN(Util.subMat(wm, 0, sensors - 1),
                Util.subMat(wm, sensors, sensors + neurons - 1),
                Util.flatten(Util.subMat(wm, sensors + neurons, sensors + neurons)));
    }

    public void intitTwoParent(VivaeController controller1, VivaeController controller2, double th) {
        double[][] wm = Util.randomArray2D(neurons, sensors + neurons + 1, -10, 10);

        FRNN mom = ((TomController) controller1).getFRNN();
        FRNN dad = ((TomController) controller2).getFRNN();

        int snum = (wm[0].length - neurons - 1);

        for (int i = 0; i < wm.length; i++) {
            for (int j = 0; j < snum; j++) {
                if (Math.random() > th) {
                    wm[i][j] = dad.getwIn()[i][j];
                } else {
                    wm[i][j] = mom.getwIn()[i][j];
                }
            }

            for (int j = snum; j < snum + neurons; j++) {
                if (Math.random() > th) {
                    wm[i][j] = dad.getwRec()[i][j - snum];
                } else {
                    wm[i][j] = mom.getwRec()[i][j - snum];
                }

            }

            int j = snum + neurons;

            if (Math.random() > th) {
                wm[i][j] = dad.getwThr()[i];
            } else {
                wm[i][j] = mom.getwThr()[i];
            }
        }

        wm = mutate(wm);

        initFRNN(Util.subMat(wm, 0, sensors - 1),
                Util.subMat(wm, sensors, sensors + neurons - 1),
                Util.flatten(Util.subMat(wm, sensors + neurons, sensors + neurons)));
    }

    public void initFRNN(double[][] wIn, double[][] wRec, double[] wThr) {
        frnn.init(wIn, wRec, wThr);
    }

    public static double[][] mutate(double[][] wm) {
        for (int i = 0; i < wm.length; i++) {
            for (int j = 0; j < wm[0].length; j++) {

                if (Math.random() < 0.10) {
                    wm[i][j] += (Math.random() - 0.5);
                }

            }

        }

        return wm;
    }

    @Override
    public double distance(VivaeController controller) {

        FRNN bot = ((TomController)controller).getFRNN();

        double dist = 0;

        for (int i = 0; i < neurons; i++) {

            for (int j = 0; j < sensors; j++) {
                dist += Math.pow(frnn.getwIn()[i][j] - bot.getwIn()[i][j], 2);
            }

            for (int j = 0; j < neurons; j++) {
                dist += Math.pow(frnn.getwRec()[i][j] - bot.getwRec()[i][j], 2);
            }

            dist += Math.pow(frnn.getwThr()[i] - bot.getwThr()[i], 2);
        }

        return Math.sqrt(dist);
    }

    @Override
    public double avgDistance(Vector<Active> bots) {

        double dist = 0;

        for (int i = 0; i < bots.size(); i++) {        
            dist += distance(bots.get(i).controller);
        }

        return dist/bots.size();
    }
}
