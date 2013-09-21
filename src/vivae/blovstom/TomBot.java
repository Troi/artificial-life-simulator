/**
 * This is VIVAE (Visual Vector Agent Environment)
 * a library allowing for simulations of agents in co-evolution
 * written as a bachelor project
 * by Petr Smejkal
 * at Czech Technical University in Prague
 * in 2008
 */
package vivae.blovstom;

import common.RND;
import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.geom.Rectangle2D;
import java.io.Serializable;
import java.lang.String;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Vector;

import net.phys2d.math.Vector2f;
import net.phys2d.raw.Body;
import net.phys2d.raw.World;
import net.phys2d.raw.shapes.Box;
import vivae.arena.parts.Passive;
import vivae.arena.parts.sensors.*;
import vivae.arena.parts.Robot;
import vivae.arena.parts.VivaeObject;
import vivae.util.Util;

/**
 * @author HKou
 */
public class TomBot extends Robot implements Serializable {

    public TomBotStats stats;
    protected double[][] sensoryData;
    public Vector<VivaeObject> inRange = new Vector<VivaeObject>();

    public TomBot(float x, float y) {
        super(x, y);
        sensors = new Vector<Sensor>();
        sensorsMap = new HashMap<Integer, Sensor>();
    }

    public TomBot(Shape shape, int layer, TomArena arena) {
        this((float) shape.getBounds2D().getCenterX(),
                (float) shape.getBounds2D().getCenterY(), arena);
    }

    public TomBot(float x, float y, TomArena arena) {
        this(x, y);

        stats = new TomBotStats(this, arena);

        diameter = 12;

        boundingCircleRadius = (float) Math.sqrt(2 * diameter * diameter) / 2;
        myNumber = getNumber();
        this.arena = arena;
        this.world = arena.getWorld();
        body = new Body("Robot", new Box(diameter, diameter), 50);
        body.setPosition((float) x, (float) y);
        body.setRotation(0);
        body.setDamping(baseDamping);
        body.setRotDamping(ROT_DAMPING_MUTIPLYING_CONST * baseDamping);
        setShape(new Rectangle2D.Double(0, 0, diameter, diameter));
        Rectangle r = getShape().getBounds();
        centerX = (float) r.getCenterX();
        centerY = (float) r.getCenterY();
    }

    public TomBot(float x, float y, TomArena arena, TomBotStats stats) {
        this(x, y);

        this.stats = stats;
        stats.setBot(this);

        diameter = 12;
       
        boundingCircleRadius = (float) Math.sqrt(2 * diameter * diameter) / 2;
        myNumber = getNumber();
        this.arena = arena;
        this.world = arena.getWorld();
        body = new Body("Robot", new Box(diameter, diameter), 50);
        body.setPosition((float) x, (float) y);
        body.setRotation(0);
        body.setDamping(baseDamping);
        body.setRotDamping(ROT_DAMPING_MUTIPLYING_CONST * baseDamping);
        setShape(new Rectangle2D.Double(0, 0, diameter, diameter));
        Rectangle r = getShape().getBounds();
        centerX = (float) r.getCenterX();
        centerY = (float) r.getCenterY();

    }

    public void setSensors(int howMany, double startingAngle, double angleIncrement,
            double maxDistance, double frictionDistance) {

        for (int i = 0; i < howMany; i++) {
            addMultiSensor(startingAngle + i * angleIncrement, maxDistance);
        }

        addMouthSensor();
    }

    public void addMultiSensor(Double angle, double maxDistance) {
        //Sensor s = new FoodDistanceSensor(this, angle, sensorNumber, maxDistance);
        Sensor s = new TomMultiSensor(this, angle, sensorNumber, maxDistance);
        sensors.add(s);
        sensorsMap.put(sensorNumber, s);
        sensorNumber++;

    }

    private void addMouthSensor() {
        Sensor s = new TomMouthSensor(this, sensorNumber, stats.size);
        sensors.add(s);
        sensorsMap.put(sensorNumber, s);

    }

    public double[][] getSensoryData() {
        double[][] data = new double[TomManager.sensorRows][sensorNumber];
        Vector<VivaeObject> allObjects = getArena().getVivaes();

        int fdi = 0, rdi = 0, di = 0;
        double v;

        Vector<VivaeObject> closeObjects = ((TomMultiSensor) sensors.get(0)).getCloseVivaes(allObjects);
        Vector<VivaeObject> closeFood = TomMultiSensor.getCloseFood(closeObjects);
        //Vector<VivaeObject> closeBadFood = TomMultiSensor.getCloseBadFood(closeObjects);
        Vector<VivaeObject> closeFriendlyRobots = TomMultiSensor.getCloseRobots(closeObjects, stats.species, true);
        Vector<VivaeObject> closeOtherRobots = TomMultiSensor.getCloseRobots(closeObjects, stats.species, false);
        Vector<VivaeObject> closeWalls = ((TomMultiSensor) sensors.get(0)).getCloseWalls(arena.getWalls());

        for (Iterator<Sensor> it = sensors.iterator(); it.hasNext();) {
            Sensor sensor = it.next();

            if (sensor instanceof TomMultiSensor) {
                ((TomMultiSensor) sensor).nullOpacity();
                data[1][fdi] = ((TomMultiSensor) sensor).getDistance(closeWalls);
                data[2][fdi] = ((TomMultiSensor) sensor).getDistance(closeFriendlyRobots);
                data[3][fdi] = ((TomMultiSensor) sensor).getDistance(closeOtherRobots);
                data[4][fdi] = ((TomMultiSensor) sensor).getDistance(closeFood);
                fdi++;
            }

            if (sensor instanceof TomMouthSensor) {
                inRange = ((TomMouthSensor) sensor).getVivaesOnSight(allObjects);
                data[0][0] = inRange.size();
            }

            data[0][1] = (1.0 / stats.energy);
            data[0][2] = this.direction;

        }

        sensoryData = data;
        return data;
    }

    @Override
    public void moveComponent() {

        stats.step();

        if (!stats.alive) {
            return;
        }

        inMotion = true;
        direction = body.getRotation();

        net.phys2d.math.ROVector2f p = body.getPosition();
        x = p.getX();
        y = p.getY();
        for (Iterator<Sensor> sIter = sensors.iterator(); sIter.hasNext();) {
            Sensor s = (Sensor) sIter.next();
            s.moveComponent();
        }

        final double distance = Util.euclideanDistance(lastX, lastY, x, y);
        final double velDist = lastVelocity - getSpeed() > 0 ? lastVelocity - getSpeed() : 0;
        if (velDist > 10) {
            crashmeter += velDist;
        }
        if (velDist > maxDeceleration) {
            maxDeceleration = velDist;
        }
        overallDeceleration += velDist;
        lastVelocity = getSpeed();

        odometer += distance;
        lastX = x;
        lastY = y;


    }

    @Override
    public AffineTransform getTranslation() {
        AffineTransform af = AffineTransform.getTranslateInstance(x - diameter / 2, y - diameter / 2);
        af.rotate(direction, centerX, centerY);
        return af;
    }

    @Override
    public void paintComponent(Graphics g) {
        Graphics2D g2 = (Graphics2D) g;
        Object hint = new Object();
        if (isAntialiased()) {
            hint = g2.getRenderingHint(RenderingHints.KEY_ANTIALIASING);
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        }

        if (isShowingSensors) {
            for (Iterator<Sensor> sIter = sensors.iterator(); sIter.hasNext();) {
                Sensor s = (Sensor) sIter.next();
                s.paintComponent(g2);
            }
        }
        translation = getTranslation();
        Color oldColor = g2.getColor();

        stats.filter(g2, x, y);
        //g2.setColor(new Color(255, 255, Math.max(0,Math.min(255, energy/4)));

        g2.fill(translation.createTransformedShape(getShape()));
        g2.setColor(Color.BLACK);
        g2.draw(translation.createTransformedShape(getShape()));

//        g2.setColor(Color.BLUE);
//        g2.drawString((int) (stats.metabolism * 100) + "", x + 10, y - 10);


        if (isShowingStatusFrame) {
            paintStatusFrame(g2);
        }
        g2.setColor(oldColor);
//        if (isAntialiased()) {
//            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, hint);
//        }
    }

    @Override
    public void accelerate(float s) {

        s = (float) (s * stats.metabolism / stats.size);

        setSpeed(body.getVelocity().length());
        s = Math.min(s, getMaxSpeed() - (float) getSpeed());
        s = Math.min(s, getMaxSpeed() - (float) getSpeed());
        float dx = (float) (s * (float) Math.cos(body.getRotation() - Math.PI / 2));
        float dy = (float) (s * (float) Math.sin(body.getRotation() - Math.PI / 2));


        body.adjustVelocity(new Vector2f(dx, dy));

    }

    @Override
    public void decelerate(float s) {

        setSpeed(body.getVelocity().length());
        s = Math.max(s, 0);
        float dx = (float) (s * (float) Math.cos(body.getRotation() - Math.PI / 2));
        float dy = (float) (s * (float) Math.sin(body.getRotation() - Math.PI / 2));
        body.adjustVelocity(new Vector2f(-dx, -dy));
    }

    @Override
    public void rotate(float radius) {

        body.adjustAngularVelocity(radius / (diameter / 12));
        this.direction = body.getRotation();
    }

    @Override
    public void eat() {

        //stats.energy-= 0.01*stats.generation;
        //stats.eating = true;


        if (inRange != null) {

            if (inRange.size() > 0) {

                if (inRange.get(0) instanceof TomFood) {
                    if (((TomFood) inRange.get(0)).value >= 0) {

                        double value = 10 * stats.size;

                        stats.energy += value;
                        ((TomFood) inRange.get(0)).value -= value;

                        stats.plantEaten += value / 10;
                    } else {
                        world.remove(inRange.get(0).getBody());
                        arena.getPassives().remove((Passive) inRange.get(0));
                        arena.getVivaes().remove((Passive) inRange.get(0));

                    }

                }



                if (inRange.get(0) instanceof TomBot) {

                    TomBot victim = (TomBot) inRange.get(0);

                    if (victim.getSpecies() != getSpecies()) {

                        double value = 40 * (stats.size / victim.stats.size);

                        stats.energy += value;
                        victim.stats.energy -= value;

                        stats.robotEaten += value / 10;
                        victim.stats.robotEaten -= Math.min(value / 10, victim.stats.robotEaten);


                    }
                }



            }
        }
    }

    @Override
    public int getNumber() {
        return robotsCounter++;


    }

    @Override
    public String getActiveName() {
        return "Robot";


    }

    @Override
    public float getAcceleration() {
        return Robot.ACCELERATION;


    }

    @Override
    public float getMaxSpeed() {
        return (float) (Robot.MAX_SPEED * stats.metabolism / stats.size);


    }

    @Override
    public float getRotationIncrement() {
        return Robot.ROTATION;


    }

    public int getGeneration() {
        return stats.generation;
    }

    public int getEnergy() {
        return (int) stats.energy;
    }

    @Override
    public String toString() {
        return "Robot " + myNumber;


    }

    @Override
    public Vector<Sensor> getSensors() {
        return sensors;


    }

    public int getSpecies() {
        return stats.species;
    }

    @Override
    public void reportObjectOnSight(Sensor s, Body b) {
        System.out.println("Object seen from sensor " + s);


    }

    @Override
    public World getWorld() {
        return world;


    }

    @Override
    public void setWorld(World world) {
        this.world = world;


    }

    @Override
    public void setShowingSensors(boolean showingSensors) {
        isShowingSensors = showingSensors;


    }

    @Override
    public void paintStatusFrame(Graphics g, int baseX, int baseY) {
        Graphics2D g2 = (Graphics2D) g;
        Color oldColor = g2.getColor();
        Composite oldComposite = g2.getComposite();
        g2.setComposite(opacityBack);
        g2.setColor(Color.BLACK);
        g2.fillRect(baseX, baseY, 100, 100);
        g2.setComposite(opacityFront);
        g2.setColor(Color.WHITE);
        g2.drawRect(baseX, baseY, 100, 100);


        if (isStatusFramePinedToPosition) {
            g2.drawLine((int) this.x, (int) this.y, baseX + 100, baseY + 100);


        }
        baseX += 5;
        baseY += 15;
        g2.drawString(String.format(getActiveName() + "  #%d", myNumber), baseX, baseY);
        baseY += STATUS_FRAME_LINE_HEIGHT;
        g2.drawString(String.format("x: %4.0f", x), baseX, baseY);
        baseY += STATUS_FRAME_LINE_HEIGHT;
        g2.drawString(String.format("y: %4.0f", y), baseX, baseY);
        baseY += STATUS_FRAME_LINE_HEIGHT;
        String s = "";


        for (int j = 0; j
                < sensoryData[0].length; j++) {
            s += String.format("%1.1f ", sensoryData[0][j]);


        }
        g2.drawString(s, baseX, baseY);
        baseY += STATUS_FRAME_LINE_HEIGHT;
        s = "";


        for (int j = 0; j
                < sensoryData[1].length; j++) {
            s += String.format("%1.1f ", sensoryData[1][j]);


        }
        g2.drawString(s, baseX, baseY);
        g2.setComposite(oldComposite);
        g2.setColor(oldColor);


    }

    public int compareTo(TomBot o) {

        return (int) ((stats.offsprings * (stats.robotAge + stats.genomAge)) - (o.stats.offsprings * (o.stats.robotAge + +stats.genomAge)));
        //return (int) (odometer - o.odometer);
    }
}

class TomBotStats implements Serializable {

    TomBot bot;
    TomArena arena;
//    public double size = 1 + Math.random() * 0.5;
//    public double metabolism = 1 + Math.random() * 0.5;
//    public double sensorLength = Math.random() + 0.5;
//    public double sensorAngle = 1.2 * Math.random() + 0.3;
//---------------------------------------------------------------
    public double size = 1;
    public double metabolism = 1;
    public double sensorLength = 1;
    public double sensorAngle = 0.8;
    public int fertility = 1;
//---------------------------------------------------------------
    public int generation = 0;
    public int offsprings = 0;
    public double maxEnergy = 1500 * size;
    public double energy = maxEnergy / 2;
    public int robotAge = 0;
    public int genomAge = 0;
    boolean alive = true;
//---------------------------------------------------------------
    static int colorDisplaied = 0;
    int species = RND.getInt(0, 1);
    double plantEaten = 0;
    double robotEaten = 0;
//---------------------------------------------------------------
    boolean eating;
//---------------------------------------------------------------

    public TomBotStats(TomBot bot, TomArena arena) {
        this(arena);

        this.bot = bot;

    }

    public TomBotStats(TomArena arena) {
        this.arena = arena;
    }

    void step() {
        if (energy <= 0) {
            alive = false;
        }

        if (!alive) {
            arena.getActives().remove(bot);
            arena.getVivaes().remove(bot);
            arena.getWorld().remove(bot.getBody());
            arena.getControllers().remove(bot.controller);
            return;
        }



        if (energy > maxEnergy && arena.getActives().size() < 250) {

            double energyGiven = maxEnergy / (fertility + 1);

            for (int i = 0; i < fertility; i++) {
                arena.manager.createCrossedRobot(bot);
                energy -= energyGiven;
                offsprings++;

            }

        }

        energy -= prize();
        robotAge++;
        genomAge++;

    }

    double prize() {
//return (metabolism * metabolism) * (size * size);

//        double sp = 0.7 + 0.3 * (bot.getSpeed() / bot.getMaxSpeed());
//        return (metabolism) * (size) * (sp * sp);

        return size * metabolism;

//        return Math.sqrt(size) * Math.sqrt(metabolism) * Math.sqrt(mouthsize);
    }

    void filter(Graphics2D g2, float x, float y) {

        int r, g, b;

        switch (colorDisplaied) {
            case 0:
                if (eating) {
                    g2.setColor(new Color(255, 255, 0));
                    eating = false;
                }

                return;
            case 1:

                b = (int) (255 * (plantEaten / (plantEaten + robotEaten + 1)));
                r = (int) (255 * (robotEaten / (plantEaten + robotEaten + 1)));
                g = 0;

                g2.setColor(Color.BLUE);
                g2.drawString((int) plantEaten + "", x + 10, y + 10);

                g2.setColor(Color.RED);
                g2.drawString((int) robotEaten + "", x + 10, y);

                g2.setColor(new Color(r, g, b));

                return;
            case 2:

                g2.setColor(Color.RED);
                g2.drawString(robotAge / 10 + "", x + 10, y - 10);

                g2.setColor(Color.BLUE);
                g2.drawString((genomAge - robotAge) / 10 + "", x + 10, y);


                r = Math.max(0, Math.min(255, (int) (robotAge / 10)));
                g = 0;
                b = Math.max(0, Math.min(255, (int) ((genomAge - robotAge) / 10)));

                g2.setColor(new Color(r, g, b));

                return;

            case 3:

                g2.setColor(Color.BLACK);
                g2.drawString(generation + "", x + 10, y + 10);

                r = Math.max(0, Math.min(255, (int) (generation * 25)));
                g = Math.max(0, Math.min(255, (int) (generation * 5)));
                b = Math.max(0, Math.min(255, (int) (generation)));

                g2.setColor(new Color(r, g, b));

                return;

            case 4:

                String s = metabolism + "   ";
                g2.setColor(Color.RED);
                g2.drawString(s.substring(0, 4) + "", x + 10, y);

//                g2.setColor(Color.ORANGE);
//                g2.drawString((int) (fertility) + "", x + 30, y);

                s = size + "   ";

                g2.setColor(Color.GREEN);
                g2.drawString(s.substring(0, 4), x + 10, y + 10);

                r = Math.max(0, Math.min(255, (int) ((metabolism) * 150)));
                g = Math.max(0, Math.min(255, (int) ((size) * 150)));
                b = 0;

                g2.setColor(new Color(r, g, b));

                return;

            case 5:

                if (bot.controller instanceof TomNeatController) {
                    int links = ((TomNeatController) bot.controller).net.getNumLinks() - 52;
                    int neurons = ((TomNeatController) bot.controller).net.getNumNeurons() - 28;

                    g2.setColor(Color.magenta);
                    g2.drawString(links + "", x + 10, y);

                    g2.setColor(Color.cyan);
                    g2.drawString(neurons + "", x + 10, y + 10);


                    r = Math.max(0, Math.min(255, (links) * 10));
                    g = Math.max(0, Math.min(255, 0 * 10));
                    b = Math.max(0, Math.min(255, (neurons) * 10));

                    g2.setColor(new Color(r, g, b));

                }
                return;

            case 6:

                g2.setColor(Color.darkGray);
                g2.drawString((int) arena.manager.threshold + "", x + 10, y + 10);

//                g2.setColor(Color.lightGray);
//                g2.drawString(arena.manager.step + "", x + 10, y + 20);


                switch (species % 10) {
                    case 0:
                        g2.setColor(Color.BLACK);
                        break;
                    case 1:
                        g2.setColor(Color.WHITE);
                        break;
                    case 2:
                        g2.setColor(Color.RED);
                        break;
                    case 3:
                        g2.setColor(Color.BLUE);
                        break;
                    case 4:
                        g2.setColor(Color.YELLOW);
                        break;
                    case 5:
                        g2.setColor(Color.MAGENTA);
                        break;
                    case 6:
                        g2.setColor(Color.CYAN);
                        break;
                    case 7:
                        g2.setColor(Color.ORANGE);
                        break;
                    case 8:
                        g2.setColor(Color.LIGHT_GRAY);
                        break;
                    case 9:
                        g2.setColor(Color.PINK);
                        break;
                    default:
                        g2.setColor(new Color((int) (Math.random() * 255), (int) (Math.random() * 255), (int) (Math.random() * 255)));
                        break;
                }

                g2.drawString(species + "", x + 10, y);




                return;
            case 7:


                g2.setColor(Color.YELLOW);
                g2.drawString((int) (energy / 10) + "", x + 10, y + 10);

                double yelow = (energy / maxEnergy)*255;

                r = (int) Math.max(0, Math.min(255, yelow ));
                g = (int) Math.max(0, Math.min(255, yelow ));
                b = 0;

                g2.setColor(new Color(r, g, b));


                return;




            default:
                g2.setColor(Color.white);
        }

//        
    }

    public void setBot(TomBot bot) {
        this.bot = bot;


    }

    public void setArena(TomArena arena) {
        this.arena = arena;


    }

    TomBotStats getCopy() {

        TomBotStats copy = new TomBotStats(arena);

        copy.genomAge = genomAge;

        copy.size = size;
        copy.metabolism = metabolism;

        copy.sensorLength = sensorLength;
        copy.sensorAngle = sensorAngle;

        copy.fertility = fertility;

        copy.generation = generation + 1;

        mutate(
                copy);

        copy.maxEnergy = 1500 * copy.size * copy.metabolism;
        copy.energy = maxEnergy / (fertility + 1);

        copy.species = species;



        return copy;


    }

    private void mutate(TomBotStats copy) {
        copy.metabolism += 0.1 * (Math.random() - 0.5);
        copy.size += 0.1 * (Math.random() - 0.5);

        copy.sensorAngle += 0.1 * (Math.random() - 0.5);
        copy.sensorLength = 2 - sensorAngle;

//        if (Math.random() > 0.95) {
//            copy.fertility++;
//        }
//        if (Math.random() < 0.05) {
//            copy.fertility--;
//        }

        copy.metabolism = Math.max(Math.min(copy.metabolism, 2), 0.5);
        copy.size = Math.max(Math.min(copy.size, 2), 0.5);
        copy.sensorAngle = Math.max(Math.min(copy.sensorAngle, 2), 0);

    }
}
