package vivae.blovstom;

import java.awt.AlphaComposite;
import java.awt.Color;
import java.awt.Composite;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.geom.AffineTransform;
import java.awt.geom.Area;
import java.awt.geom.Rectangle2D;
import java.util.Vector;
import net.phys2d.raw.Body;
import net.phys2d.raw.shapes.Box;
import vivae.arena.parts.Active;
import vivae.arena.parts.VivaeObject;
import vivae.arena.parts.Fixed;
import vivae.blovstom.TomFood;
import vivae.arena.parts.Passive;
import vivae.arena.parts.sensors.Sensor;

/**
 * @author Petr Smejkal
 */
public class TomMouthSensor extends Sensor {

    protected Active owner;
    protected Body ownerBody;
    protected float ray_length = 10f;
    protected float ray_width = 10f;
    protected float angle = 0f;
    protected AlphaComposite opacityOfRay = AlphaComposite.getInstance(AlphaComposite.SRC_OVER, 0.15f);
    protected boolean isRayTransparent = true;
    protected int sensorNumber = 0;
    protected int sensorX, sensorY;

    /**
     * This method removes all VivaeObjects that are further from owner of this Sensor
     * than length of the Sensor is.
     * @param objects Vector of all VivaeObjects that are checked for distance from owner of this Sensor.
     * @param walls Vector of walls that can contain enclosing walls in Arena.
     * @return new Vector of VivaeObjects that are close enough to be in range of Sensor.
     */
    public Vector<VivaeObject> getCloseVivaes(Vector<VivaeObject> objects, Vector<Fixed> walls) {

        Vector<VivaeObject> closeObjects = new Vector<VivaeObject>();

        for (VivaeObject vivae : objects) {
            if (vivae.getBoundingCircleRadius() + ray_length > vivae.getBody().getPosition().distance(ownerBody.getPosition())) {

                //if (vivae instanceof TomFood) {
                    closeObjects.add(vivae);
                //}


            }
        }

        return closeObjects;



    }

    /**
     * Method intersects area of Sensor and all VivaeObjects and returns those that
     * have non-zero intersection.
     * @param objects Vector of VivaeObjects that are checked for collision with the body of Sensor.
     * @return Vector of VivaeObjects that are in collision with the body of Sensor.
     */
    public Vector<VivaeObject> getVivaesOnSight(Vector<VivaeObject> objects) {
        Vector<VivaeObject> objectsOnSight = new Vector<VivaeObject>();
        //for (VivaeObject vivaeObject : objects) {
        for (VivaeObject vivaeObject : getCloseVivaes(objects, owner.getArena().getWalls())) {
            if (vivaeObject != this.owner) {
                Area actArea = (Area) vivaeObject.getArea().clone();
                actArea.intersect(this.getArea());
                if (!actArea.isEmpty()) {
                    objectsOnSight.add(vivaeObject);
                }
                //             if(vivaeObject instanceof roboneat.RoboNeatRobot)System.out.println("robot seen by"+this.owner);
            }
        }
        return objectsOnSight;
    }

    public TomMouthSensor(Active owner, double angle, int number) {
        this(owner, number);
        setAngle((float) angle);
    }

    public TomMouthSensor(Active owner) {
        super(owner);
        this.ownerBody = owner.getBody();
        this.owner = owner;
        //this.sensorNumber  = number();
    }

    public TomMouthSensor(Active owner, int number) {
        super(owner);
        this.owner = owner;
        this.ownerBody = owner.getBody();
        this.sensorNumber = number;
        body = new Body("Sensor", new Box(ray_length, ray_width), 50f);
        body.addExcludedBody(owner.getBody());
        body.setDamping(baseDamping);
        body.setRotDamping(ROT_DAMPING_MUTIPLYING_CONST * baseDamping);
        setShape(new Rectangle2D.Double(0, 0, ray_length, ray_width));
    }

    public TomMouthSensor(Active owner, int number, double size) {
        super(owner);
        this.owner = owner;
        this.ownerBody = owner.getBody();
        this.sensorNumber = number;

        ray_length = (float) ((size / 2) * ray_length)+5;
        ray_width = (float) ((size / 1) * ray_width);

        body = new Body("Sensor", new Box(ray_length, ray_width), 50f);
        body.addExcludedBody(owner.getBody());
        body.setDamping(baseDamping);
        body.setRotDamping(ROT_DAMPING_MUTIPLYING_CONST * baseDamping);
        setShape(new Rectangle2D.Double(0, 0, ray_length, ray_width));
    }

    @Override
    public void moveComponent() {
        inMotion = true;
        direction = owner.getDirection() - (float) Math.PI / 2;
        direction += angle;
        net.phys2d.math.ROVector2f op = ownerBody.getPosition();
        x = op.getX();
        y = op.getY();
        float newX = (float) (x + (ray_length / 2) * Math.cos(direction));
        float newY = (float) (y + (ray_length / 2) * Math.sin(direction));
        body.setPosition(newX, newY);
        body.setRotation(direction);
    }

    public AffineTransform getTranslation() {
        AffineTransform translation = AffineTransform.getTranslateInstance(x, y - ray_width / 2);
        translation.rotate(direction, 0, ray_width / 2);
        return translation;
    }

    @Override
    public void paintComponent(Graphics g) {
        Graphics2D g2 = (Graphics2D) g;
        Object hint = new Object();
        if (isAntialiased()) {
            hint = g2.getRenderingHint(RenderingHints.KEY_ANTIALIASING);
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        }
        translation = getTranslation();
        Color oldColor = g2.getColor();
        Composite oldComposite = g2.getComposite();
        if (isRayTransparent) {
            g2.setComposite(opacityOfRay);
        }
        g2.setColor(Color.RED);
        g2.draw(translation.createTransformedShape(getShape()));
        g2.fill(translation.createTransformedShape(getShape()));
        g2.setComposite(opacityFront);
        g2.setComposite(oldComposite);
        g2.setColor(oldColor);
        if (isAntialiased()) {
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, hint);
        }
    }

    @Override
    public void accelerate(float speed) {
        // TODO Auto-generated method stub
    }

    @Override
    public void decelerate(float speed) {
        // TODO Auto-generated method stub
    }

    @Override
    public float getAcceleration() {
        return 0;
    }

    @Override
    public String getActiveName() {
        // TODO Auto-generated method stub
        return "Sensor";
    }

    @Override
    public float getMaxSpeed() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public int getNumber() {
        // TODO Auto-generated method stub
        return 0;

    }

    @Override
    public float getRotationIncrement() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public void rotate(float radius) {
        // TODO Auto-generated method stub
    }

    public float getAngle() {
        return angle;
    }

    public void setAngle(float angle) {
        this.angle = angle;
    }

    public boolean isRayTransparent() {
        return isRayTransparent;
    }

    public void setRayTransparent(boolean isRayTransparent) {
        this.isRayTransparent = isRayTransparent;
    }

    @Override
    public String toString() {
        return "Sensor " + sensorNumber + " on " + owner.toString();
    }

    @Override
    public void reportObjectOnSight(Sensor s, Body b) {
        // TODO Auto-generated method stub
    }
}
