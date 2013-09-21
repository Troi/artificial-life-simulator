package vivae.blovstom;

import common.net.linked.Net;
import common.stats.StatSeries;
import java.util.ArrayList;
import java.util.Vector;
import neat.GlobalInnovation;
import neat.Population;
import vivae.arena.parts.Active;
import vivae.arena.parts.Fixed;
import vivae.arena.parts.Passive;
import vivae.controllers.RobotWithSensorController;

public class TomManager {

    TomArena arena;

    public TomManager(TomArena arena) {
        this.arena = arena;

        TomController.setParameters(neurons, sensors * sensorRows, -5, +5);
        TomNeatController.setParameters(outputs, sensors * sensorRows, -7, +7);

    }
    //-------------------------------------------------------------------------
    static double maxDistance = 60;
    static double frictionDistance = 25;
    static int sensors = 5; //
    static int sensorRows = 5;
    static int neurons = 15;
    static int outputs = 2;
    int typ = 2;

    ArrayList<Vector<Active>> pop;
    public double threshold = 6;
    public boolean speciated = true;
    double step = 0.01;
    boolean up = true;

    public void createRandomRobot() {
        switch (typ) {
            case 1:
                randomFrnnRobot();
                break;
            case 2:
                randomNeatRobot();
                break;
        }
    }

    public void createCrossedRobot(TomBot bot) {

        //int i = Math.random() * pop.get(bot.stats.species).size();

        int sp = bot.stats.species;
        sp = Math.min(pop.size() - 1, sp);

//        int rbt = (int) (Math.random() * pop.get(sp).size());
//        rbt = Math.min(pop.get(sp).size()-1, rbt);

        int rbt = 0;

        TomBot bot2 = (TomBot) pop.get(sp).get(rbt);

        switch (typ) {
            case 1:
                twoParentFrnnRobot(bot, bot2, 0.8, true);
                break;
            case 2:
                twoParentNeatRobot(bot, bot2, 0.25, false);
                break;

        }
    }

    public void createCrossedRobot() {

        int sz = 100;
        int sp = 0;

        for (int i = 0; i < pop.size(); i++) {

            if (pop.get(i).size() < sz && pop.get(i).size() > 0) {
                sz = pop.get(i).size();
                sp = i;
            }

        }

        int rbt = (int) (Math.random() * pop.get(sp).size());

        TomBot bot1 = (TomBot) pop.get(sp).get(rbt);



        createCrossedRobot(bot1);

    }

    //--------------------------------------------------------------------------
    public void randomFrnnRobot() {
        TomController frnnc = new TomController();
        frnnc.intitRandom();

        TomBot agent;
        agent = new TomBot((int) (Math.random() * (arena.screenWidth - 100)) + 50,
                (int) (Math.random() * (arena.screenHeight - 100)) + 50, arena);

        createRobot(frnnc, agent);
    }

    public void oneParentFrnnRobot(TomBot bot, boolean rand) {
        TomController frnnc = new TomController();
        frnnc.intitOneParent(bot.controller);

        TomBot agent;
        int xx, yy;

        if (rand) {
            xx = (int) (Math.random() * (arena.screenWidth - 100)) + 50;
            yy = (int) (Math.random() * (arena.screenHeight - 100)) + 50;
        } else {
            xx = (int) Math.max(Math.min(bot.getX() - 50 + 100 * Math.random(), arena.screenWidth), 0);
            yy = (int) Math.max(Math.min(bot.getY() - 50 + 100 * Math.random(), arena.screenHeight), 0);
        }

        agent = new TomBot(xx, yy, arena, bot.stats.getCopy());

        createRobot(frnnc, agent);
    }

    public void twoParentFrnnRobot(TomBot bot1, TomBot bot2, double th, boolean rand) {
        TomController frnnc = new TomController();
        frnnc.intitTwoParent(bot1.controller, bot2.controller, th);

        TomBot agent;
        int xx, yy;

        if (rand) {
            xx = (int) (Math.random() * (arena.screenWidth - 100)) + 50;
            yy = (int) (Math.random() * (arena.screenHeight - 100)) + 50;
        } else {
            xx = (int) Math.max(Math.min(bot1.getX() - 50 + 100 * Math.random(), arena.screenWidth), 0);
            yy = (int) Math.max(Math.min(bot1.getY() - 50 + 100 * Math.random(), arena.screenHeight), 0);
        }

        agent = new TomBot(xx, yy, arena, bot1.stats.getCopy());

        createRobot(frnnc, agent);
    }

    //--------------------------------------------------------------------------
    public void randomNeatRobot() {
        TomNeatController neat = new TomNeatController();
        neat.intitRandom();

        TomBot agent;
        agent = new TomBot((int) (Math.random() * (arena.screenWidth - 100)) + 50,
                (int) (Math.random() * (arena.screenHeight - 100)) + 50, arena);

        createRobot(neat, agent);
    }

    public void oneParentNeatRobot(TomBot bot, boolean rand) {
        TomNeatController neat = new TomNeatController();
        neat.intitOneParent(bot.controller);

        TomBot agent;
        int xx, yy;

        if (rand) {
            xx = (int) (Math.random() * (arena.screenWidth - 100)) + 50;
            yy = (int) (Math.random() * (arena.screenHeight - 100)) + 50;
        } else {
            xx = (int) Math.max(Math.min(bot.getX() - 50 + 100 * Math.random(), arena.screenWidth), 0);
            yy = (int) Math.max(Math.min(bot.getY() - 50 + 100 * Math.random(), arena.screenHeight), 0);
        }

        agent = new TomBot(xx, yy, arena, bot.stats.getCopy());

        createRobot(neat, agent);
    }

    public void twoParentNeatRobot(TomBot bot1, TomBot bot2, double th, boolean rand) {
        TomNeatController neat = new TomNeatController();

        if (Math.random() < th) {
            neat.intitOneParent(bot1.controller);
        } else {
            neat.intitTwoParent(bot1.controller, bot2.controller);
        }

        TomBot agent;
        int xx, yy;

        if (rand) {
            xx = (int) (Math.random() * (arena.screenWidth - 100)) + 50;
            yy = (int) (Math.random() * (arena.screenHeight - 100)) + 50;
        } else {
            xx = (int) Math.max(Math.min(bot1.getX() - 50 + 100 * Math.random(), arena.screenWidth), 0);
            yy = (int) Math.max(Math.min(bot1.getY() - 50 + 100 * Math.random(), arena.screenHeight), 0);
        }

        agent = new TomBot(xx, yy, arena, bot1.stats.getCopy());

        createRobot(neat, agent);
    }

    //--------------------------------------------------------------------------
    public void createRobot(RobotWithSensorController controller, TomBot agent) {

        int snum = sensors * sensorRows;

        double sangle = -Math.PI / 2;
        double ai = Math.PI / (sensors - 1);

        sangle = sangle * agent.stats.sensorAngle;
        ai = ai * agent.stats.sensorAngle;

        arena.registerController(agent, controller);

        agent.setSensors(sensors, sangle, ai, maxDistance * agent.stats.sensorLength, frictionDistance);

        agent.setAntialiased(true);
        arena.getWorld().add(agent.getBody());
        arena.addActive(agent);

        distinctSpecies();
    }

    public void distinctSpecies() {

        pop = new ArrayList<Vector<Active>>();
        Vector<Active> actives = arena.getActives();
        if (actives.size() < 1) {
            return;
        }

        sortRobots();


        pop.add(new Vector<Active>());
        pop.get(0).add(actives.get(0));

        for (int i = 1; i < actives.size(); i++) {

            double dist = actives.get(i).controller.avgDistance(pop.get(0));

            int sp = 0;

            for (int j = 1; j < pop.size(); j++) {

                double d = actives.get(i).controller.avgDistance(pop.get(j));

                if (d < dist) {
                    dist = d;
                    sp = j;
                }
            }

            if (dist < threshold) {
                pop.get(sp).add(actives.get(i));
            } else {
                sp = pop.size();
                pop.add(new Vector<Active>());
                pop.get(sp).add(actives.get(i));

            }
            ((TomBot) actives.get(i)).stats.species = sp;
        }

        if (pop.size() < 10 && pop.size() > 5) {
            speciated = true;
        } else {
            speciated = false;

            if (pop.size() < 7) {

//                if (!up) {
//                    step += Math.min(step / 2, 1);
//                } else {
//                    step = step / 2;
//                }

                up = false;
                threshold -= step;
            } else {

//                if (up) {
//                    step += Math.min(step / 2, 1);
//                } else {
//                    step = step / 2;
//                }

                up = true;
                threshold += step;
            }
        }

    }

    public void createFood() {

        Passive newFood;
        int xx = 0, yy = 0;

        boolean flag = true;

        Vector<Fixed> walls = arena.getWalls();

        while (flag) {
            flag = false;
            xx = (int) ((Math.random() * arena.screenWidth));
            yy = (int) ((Math.random() * arena.screenHeight));

            for (Fixed fixed : walls) {
                if (fixed.getShape().contains(xx, yy)) {
                    flag = true;
                }
            }
        }

        newFood = new TomFood(xx, yy) {

            @Override
            protected float getMass() {
                return 1;
            }
        };

        arena.addPassive(newFood);
        arena.getWorld().add(newFood.getBody());

    }

    public void sortRobots() {


        TomBot a, b;
        Vector<Active> actives = arena.getActives();


        if (actives.size() < 2) {
            return;
        }

        for (int i = 0; i < actives.size() - 1; i++) {
            a = (TomBot) actives.get(i);
            b = (TomBot) actives.get(i + 1);

            if (a.compareTo(b) < 0) {
                actives.remove(i + 1);
                actives.add(i, b);
            }

        }
    }

    public int minPopNum() {

        int sp = 0, sz = pop.get(0).size();

        for (int i = 0; i < pop.size(); i++) {

            if (pop.get(i).size() < sz && pop.get(i).size() > 0) {
                sz = pop.get(i).size();
                sp = i;
            }

        }

        return sz;
    }
}
