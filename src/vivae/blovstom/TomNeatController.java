package vivae.blovstom;

import common.net.linked.Net;
import java.util.Vector;
import neat.Genome;
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
public class TomNeatController extends RobotWithSensorController {

    protected Genome gen;
    protected Net net;
    static int id = 0;
    
    public static double activationMin = -0.5, activationRange = 1.5;

    @Override
    public void moveControlledObject() {

        if (robot instanceof TomBot) {

            double[] input = Util.flatten(((TomBot) robot).getSensoryData());

            net.loadInputs(input);
            net.reset();
            activate(net);

            double[] eval = net.getOutputValues();

           
//
            for (int i = 0; i < eval.length; i++) {
               eval[i] = activationRange*eval[i] + activationMin;

            }

            double lWheel = eval[0];
            double rWheel = eval[1];

            double angle;
            double acceleration = 5.0 * (lWheel + rWheel);
//            if (acceleration < 0) {
//                acceleration = 0; // negative speed causes problems, why?
//            }
            double speed = Math.abs(robot.getSpeed() / robot.getMaxSpeed());
            speed = Math.min(Math.max(speed, -1), 1);
            if (rWheel > lWheel) {
                angle = 10 * (1.0 - speed);
            } else {
                angle = -10 * (1.0 - speed);
            }

            robot.rotate((float) angle);
            robot.accelerate((float) acceleration);

           
            robot.eat();

        }

    }

    public void activate(Net on) {
        for (int i = 0; i < 5; i++) {
            on.activate();
        }
    }

    public void setGenom(Genome g) {
        this.gen = g;
    }

    public Genome getGenom() {
        return gen;
    }
    static int outputs;
    static int sensors;
    static int min;
    static int max;

    static void setParameters(int outputs, int sensors, int min, int max) {

        TomNeatController.outputs = outputs;
        TomNeatController.sensors = sensors;
        TomNeatController.min = min;
        TomNeatController.max = max;

        int[] h = {};
        Net n = new Net(0);
        n.createFeedForward(sensors, h, outputs);
        Genome.initInov(n);
    }

    void intitRandom() {
        int[] h = {};
        net = new Net(id);
        net.createFeedForward(sensors, h, outputs);
        net.randomizeWeights(min, max);
        id++;
        Genome mom = new Genome(net);

        double mutateAddLink = 0.2;
        double mutateAddNeuron = 0.1;
        double r = Math.random();

        if (r < mutateAddNeuron) {
            gen = mom.mutateAddNeuron();
        } else if (r < mutateAddLink + mutateAddNeuron) {
            gen = mom.mutateAddLink();
        } else {
            gen = mom;
        }

        net = gen.getNet();

    }

    public void intitOneParent(VivaeController controller) {

        Genome mom = ((TomNeatController) controller).getGenom();

        double mutateAddLink = 0.2;
        double mutateAddNeuron = 0.1;
        double r = Math.random();

        if (r < mutateAddNeuron) {
            gen = mom.mutateAddNeuron();
        } else if (r < mutateAddLink + mutateAddNeuron) {
            gen = mom.mutateAddLink();
        } else { // non-structural mutations
            gen = mom.mutateWeights();
            gen.mutateToggleEnabled();
            gen.mutateActivation();
            /** TODO maybe call from mutateWeight (DelphiNEAT) */
        }

        net = gen.getNet();

    }

    public void intitTwoParent(VivaeController controller1, VivaeController controller2) {


        Genome mom = ((TomNeatController) controller1).getGenom();
        Genome dad = ((TomNeatController) controller2).getGenom();

        gen = mom.mateMultipoint(dad);

        net = gen.getNet();

    }

    @Override
    public double distance(VivaeController controller) {
        Genome bot = ((TomNeatController) controller).getGenom();
        return gen.distance(bot);
    }

    @Override
    public double avgDistance(Vector<Active> bots) {
        double dist = 0;





        for (int i = 0; i
                < bots.size(); i++) {
            dist += distance(bots.get(i).controller);




        }

        return dist / bots.size();


    }
}
