/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package robot.eye;

import java.awt.Image;
import javax.swing.JComponent;

/**
 *
 * @author root
 */
public class EyeImage {

    protected int width;
    protected int height;
    
    public static EyeImage createEyeImage(int width, int height,
            String imageType, byte[] data) {

        return new EyeRGBImage(width,height, data);
            
    }

    public EyeImage(int width, int height) {
        this.width = width;
        this.height = height;
    }

    public Image getImage(JComponent c) {
        return null;
    }

}
