/**
 * This is VIVAE (Visual Vector Agent Environment)
 * a library allowing for simulations of agents in co-evolution 
 * written as a bachelor project 
 * by Petr Smejkal
 * at Czech Technical University in Prague
 * in 2008
 */
package vivae.blovstom;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.geom.AffineTransform;
import java.awt.geom.Ellipse2D;
import java.io.Serializable;
import net.phys2d.raw.Body;
import net.phys2d.raw.shapes.Circle;
import vivae.arena.parts.Movable;

/**
 * @author Petr Smejkal
 */
public abstract class TomFood extends Movable implements Serializable {

    protected abstract float getMass();
    public int diameter = 10;
    public int value = 1000;

    public TomFood(float x, float y) {
        super(x, y);

        shape = new Ellipse2D.Double(0, 0, diameter, diameter);
        boundingCircleRadius = diameter;
        body = new Body("Food", new Circle(diameter / 2), 100f);
        // puck placed to the topleft corner - position in the svg file, not nice,
        // but fitness works correctly (position is not changed)
        body.setPosition(x, y);
        body.setRotation(0);
        setBaseDamping(0.3f);
        body.setDamping(baseDamping);
        body.setRotDamping(ROT_DAMPING_MUTIPLYING_CONST * baseDamping);
    }

    @Override
    public AffineTransform getTranslation() {
        return AffineTransform.getTranslateInstance(x - diameter / 2, y - diameter / 2);
    }

    @Override
    public void paintComponent(Graphics g) {


        Graphics2D g2 = (Graphics2D) g;
        Object hint = new Object();

//        if (isAntialiased()) {
//            hint = g2.getRenderingHint(RenderingHints.KEY_ANTIALIASING);
//            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
//        }
        
        translation = getTranslation();
        Color oldColor = g2.getColor();
        g2.setColor(new Color(0, Math.max(0, Math.min(255, value / 6)), 0));
        g2.fill(translation.createTransformedShape(getShape()));
        g2.setColor(Color.BLACK);
        g2.draw(translation.createTransformedShape(getShape()));
        g2.setColor(oldColor);

//        if (isAntialiased()) {
//            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, hint);
//        }
    }
}
