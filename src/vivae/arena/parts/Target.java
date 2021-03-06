/**
 * HKou
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
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.awt.image.BufferedImage;
import java.io.FileInputStream;
import java.io.InputStream;
import javax.imageio.ImageIO;
import vivae.blovstom.TomArena;

/**
 * @author Jan Koutnik
 */
public class Target extends Surface {

    private final String TILE_FILE_NAME = "cfg/vivae/tiles/road-tile2.jpg";
    protected static float FRICTION = 0.5f;

    private TexturePaint texture;
    private boolean isTextureLoaded;
    private boolean isTextureEnabled = false;

    public Target(float x, float y, Shape shape){
        this(x,y, shape, 1);
    }
    
    public Target(Shape shape, int layer, TomArena arena, Point2D.Float[] points) {
        this((float) shape.getBounds2D().getMinX(), (float) shape.getBounds2D().getMinY(), shape, layer);
    }
    
    public Target(Shape shape, int layer, TomArena arena) {
        this((float) shape.getBounds2D().getMinX(), (float) shape.getBounds2D().getMinY(), shape, layer);
    }


    public Target(float x, float y, Shape shape, int level) {
        super(x, y, shape, level);
        isTextureLoaded = false;
        this.area = new Area(shape);
        InputStream in;
        try {
            in = new FileInputStream(TILE_FILE_NAME);
            BufferedImage bi = ImageIO.read(in);
            texture = new TexturePaint(bi, new Rectangle2D.Double(0,0,bi.getWidth(),bi.getHeight()));
            isTextureLoaded = true;
        } catch (Exception e) {
            isTextureLoaded = false;
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
        else g2.setColor(new Color(1f,0f,0f));
        g2.draw(getArea());
        g2.fill(getArea());
        g2.setPaint(oldPaint);
        g2.setColor(oldColor);
        if(isAntialiased()) g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING,hint);
    }

    @Override
    public float getFriction() {
        return FRICTION;
    }

    @Override
    public String toString(){
        return "Target Surface VivaeObject at " + "[" + getX() + ", " + getY() + "]";
    }

}