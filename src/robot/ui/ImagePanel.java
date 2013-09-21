/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package robot.ui;

import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.geom.AffineTransform;
import javax.swing.JComponent;

/**
 *
 * @author mirek
 */
public class ImagePanel extends JComponent {
    private Image image;

    public void setImage(Image image) {
        this.image = image;
        repaint();
    }

    public Image getImage() {
        return image;
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g.create();
        if (image != null) {
            AffineTransform transf = AffineTransform.getScaleInstance(0.5, 0.5);
            g2d.translate(10, 10);
            g2d.setPaintMode();
            g2d.drawImage(image, transf, this);
    
        }
    }
}
