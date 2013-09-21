/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package robot.eye;

import java.awt.Image;
import java.awt.Toolkit;
import java.awt.image.ColorModel;
import java.awt.image.MemoryImageSource;
import javax.swing.JComponent;

/**
 *
 * @author root
 */
public class EyeRGBImage extends EyeImage {
        private int[] data;
        
        private class RGBColorModel extends ColorModel {

        public RGBColorModel() {
            super(8);
        }

        @Override
        public int getRed(int pixel) {
            return (pixel >> 16) & 0xFF;
        }

        @Override
        public int getGreen(int pixel) {
            return (pixel >> 8) & 0xFF;
        }

        @Override
        public int getBlue(int pixel) {
            return (pixel) & 0xFF;
        }

        @Override
        public int getAlpha(int pixel) {
            return (pixel >> 24) & 0xFF;
        }

    }

    public EyeRGBImage(int w, int h, byte[] data) {
        super(w, h);
        this.data = new int[w*h];
        for(int i = 0; i < h; i++) {
            for(int j = 0; j < w; j++) {
                int index = w*i+j;
                this.data[index] =
                        0xFF000000 |
                        (data[index * 3 + 0] & 0xFF) |
                        ((data[index * 3 + 1] & 0xFF) << 8) |
                        ((data[index * 3 + 2] & 0xFF) << 16);
            }
        }
    }

    @Override
    public Image getImage(JComponent g) {
        return Toolkit.getDefaultToolkit().createImage(
            new MemoryImageSource(width, height, new RGBColorModel(),
                data, 0, width));
    }

}
