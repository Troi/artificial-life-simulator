/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package robot.eye;

import java.awt.Image;
import java.awt.Toolkit;
import java.awt.image.ColorModel;
import java.awt.image.MemoryImageSource;

public class EyeDistanceImage extends EyeImage {
    
    private float distMin = 0;
    private float distMax = 1;
    private byte[] data;

    private class DistanceColorModel extends ColorModel {
        
        public DistanceColorModel() {
            super(32);
        }    
            
        @Override
        public int getRed(int pixel) {
            float v = Float.intBitsToFloat(pixel);
            return Math.max(Math.min((int)((v - distMin)/(distMax-distMin)*255.0), 255),255);
        }

        @Override
        public int getGreen(int pixel) {
            float v = Float.intBitsToFloat(pixel);
            return Math.max(Math.min((int)((v - distMin)/(distMax-distMin)*255.0), 255), 255);
        }

        @Override
        public int getBlue(int pixel) {
            float v = Float.intBitsToFloat(pixel);
            return Math.max(0, Math.min((int)((v - distMin)/(distMax-distMin)*255.0), 255));
        }

        @Override
        public int getAlpha(int pixel) {
            return 0;
        }
        
    }

    private float bytesToFloat(byte[] data, int offset) {
        int v = data[offset] |
                data[offset+1] << 8  |
                data[offset+2] << 16 |
                data[offset+3] << 24;
        return Float.intBitsToFloat(v);
    }

    public EyeDistanceImage(int w, int h, byte[] data) {
        super(w,h);
        this.data = data;
    }

    public float getValue(int x, int y) {
        return bytesToFloat(data, (y * width + x) * 4);
    }
    
    public Image getImage() {
        return Toolkit.getDefaultToolkit().createImage(
                new MemoryImageSource(width, width,
                    new DistanceColorModel(),
                    data, 0, width * 4));
    }
}
