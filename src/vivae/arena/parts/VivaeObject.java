/**
 * This is VIVAE (Visual Vector Agent Environment)
 * a library allowing for simulations of agents in co-evolution 
 * written as a bachelor project 
 * by Petr Smejkal
 * at Czech Technical University in Prague
 * in 2008
 */
package vivae.arena.parts;

import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.geom.Area;
import net.phys2d.raw.Body;

/**
 * @author Petr Smejkal
 */
public abstract class VivaeObject extends ArenaPart implements Comparable<VivaeObject>{

    protected Body body;
    protected AffineTransform translation;
    protected float baseDamping = 1f;
    protected float direction;
    protected double speed;
    public float centerX, centerY;
    public static final int ROT_DAMPING_MUTIPLYING_CONST = 1000;
    protected boolean isBlinking = false;
    protected boolean isBlinkedNow = false;
    protected float boundingCircleRadius = 0f;
    public boolean inMotion = true;
    protected Area area = null;
    protected double lastX;
    protected double lastY;

    public abstract AffineTransform getTranslation();

    VivaeObject(float x, float y) {
        super(x, y);
        this.speed = 0;
        this.direction = (float) Math.PI;
        lastX = x;
        lastY = y;
    }

    @Override
    public Area getArea() {
        if (inMotion) {
            if (getShape() != null) {
                area = new Area(getTranslation().createTransformedShape(getShape()));
                inMotion = false;
            } else {
                return null;
            }
        }
        return area;
    }

    public Shape getTransformedShape() {
        return getTranslation().createTransformedShape(getShape());
    }

    public void moveComponent() {
        direction = body.getRotation();
        x = body.getPosition().getX();
        y = body.getPosition().getY();

        int height = 800;
        int width  = 1200;

//        if(x<0){
//            x+=width;
//        }
//
//        if(y<0){
//            y+=height;
//        }
//
//        if(x>height){
//           x-= width;
//        }
//
//        if(x<0){
//            y-=height;
//
//        }
//
//        body.setPosition(x,y );

        speed = body.getVelocity().length();
        inMotion = true;
    }

    public void paintComponent(Graphics g, boolean isBlinkingEnable) {
        if ((isBlinkingEnable && (isBlinking && isBlinkedNow) || !isBlinking) || !isBlinkingEnable) {
            paintComponent(g);
        }
        isBlinkedNow = !isBlinkedNow;
    }

    public void setDamping(float damping) {
        body.setDamping(baseDamping + damping);
    }

    public float getDamping() {
        return body.getDamping();
    }

    public Body getBody() {
        return body;
    }

    public void setBody(Body body) {
        this.body = body;
    }

    public void setTranslation(AffineTransform translation) {
        this.translation = translation;
    }

    public float getBaseDamping() {
        return baseDamping;
    }

    public void setBaseDamping(float baseDamping) {
        this.baseDamping = baseDamping;
    }

    public float getDirection() {
        return direction;
    }

    public void setDirection(float direction) {
        this.direction = direction;
    }

    public double getSpeed() {
        return speed;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public boolean isBlinking() {
        return isBlinking;
    }

    public void setBlinking(boolean isBlinking) {
        this.isBlinking = isBlinking;
    }

    public float getCenterX() {
        return centerX;
    }

    public float getCenterY() {
        return centerY;
    }

    public void setCenterX(float centerX) {
        this.centerX = centerX;
    }

    public void setCenterY(float centerY) {
        this.centerY = centerY;
    }

    public float getBoundingCircleRadius() {
        return boundingCircleRadius;
    }

    public int compareTo(VivaeObject o) {
      return (int)(getBody().getPosition().getX() - o.getBody().getPosition().getX());
    }


}
