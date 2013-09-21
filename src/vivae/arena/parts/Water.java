/**
 * This is VIVAE (Visual Vector Agent Environment)
 * a library allowing for simulations of agents in co-evolution 
 * written as a bachelor project 
 * by Petr Smejkal
 * at Czech Technical University in Prague
 * in 2008
 */

package vivae.arena.parts;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Paint;
import java.awt.RenderingHints;
import java.awt.Shape;
import java.awt.TexturePaint;
import java.awt.geom.Area;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.awt.image.BufferedImage;
import java.io.FileInputStream;
import java.io.InputStream;
import javax.imageio.ImageIO;
import vivae.blovstom.TomArena;

/**
 * @author Petr Smejkal
 */
public class Water extends Surface {
    private final String TILE_FILE_NAME = "cfg/vivae/tiles/water-tile.jpg";
    protected static float FRICTION = 6.5f;

    private TexturePaint texture;
    private boolean isTextureLoaded;
    private boolean isTextureEnabled = false;

    public Water(float x, float y, Shape shape){
        this(x,y, shape, 1);
    }
    
    public Water(Shape shape, int layer, TomArena arena, Point2D.Float[] points) {
        this((float) shape.getBounds2D().getMinX(), (float) shape.getBounds2D().getMinY(), shape, layer);
    }
    
    public Water(Shape shape, int layer, TomArena arena) {
        this((float) shape.getBounds2D().getMinX(), (float) shape.getBounds2D().getMinY(), shape, layer);
    }

    public Water(float x, float y, Shape shape, int level) {
        super(x, y, shape, level);
        isTextureLoaded = false;
        InputStream in;
        if(isTextureEnabled){
            try {
                in = new FileInputStream(TILE_FILE_NAME);
                BufferedImage bi = ImageIO.read(in);
                texture = new TexturePaint(bi, new Rectangle2D.Double(0,0,bi.getWidth(),bi.getHeight()));
                isTextureLoaded = true;
            } catch (Exception e) {
                // TODO Auto-generated catch block
                isTextureLoaded = false;
                e.printStackTrace();
            }
        }
        isTextureLoaded = false;
    }

    @Override
    public void paintComponent(Graphics g) {
        Graphics2D g2 = (Graphics2D) g;
        Paint oldPaint = g2.getPaint();
        Color oldColor = g2.getColor();
        Object hint = new Object();
        if(isAntialiased()){
            hint = g2.getRenderingHint(RenderingHints.KEY_ANTIALIASING);
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING,RenderingHints.VALUE_ANTIALIAS_ON);	
        }
        if(isTextureLoaded && isTextureEnabled) g2.setPaint(texture);

        else g2.setColor(new Color(220,225,250));
        g2.draw(getArea());
        g2.fill(getArea());
        g2.setPaint(oldPaint);
        g2.setColor(oldColor);
        if(isAntialiased()) g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING,hint);
    }

    @Override
    public float getFriction() {
        // TODO Auto-generated method stub
        return FRICTION;
    }

    @Override
    public String toString(){
        return "Water Surface VivaeObject at " + "[" + getX() + ", " + getY() + "]";
    }
}