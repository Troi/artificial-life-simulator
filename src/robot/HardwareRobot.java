/*
 * Tato trida reprezentuje rozhrani pro realneho robota
 * 
 */
package robot;

import java.io.BufferedInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.util.HashMap;
import java.util.Locale;
import java.util.Map;
import java.util.logging.Level;
import java.util.logging.Logger;
import robot.eye.EyeImage;

/**
 *
 * @author mirek
 */
public class HardwareRobot implements IHardwareRobotInterface {

    public static final String EYE_IMAGE_LEFT = "leftEye";
    public static final String EYE_IMAGE_RIGHT = "rightEye";
    public static final String EYE_IMAGE_DEPTH_MAP = "depthMap";
    public static final String RESP_DONE_STR = "done";
    public static final String RESP_OUT_OF_RANGE_STR = "out_of_range";
    public static final String RESP_UNKNOWN_COMMAND_STR = "unknown_command";
    public static final String RESP_ALERT_LEFT_STR = "alert_left";
    public static final String RESP_ALERT_RIGHT_STR = "alert_right";
    public static final String RESP_ALERT_BACK_STR = "alert_back";
    public static final String RESP_ALERT_FRONT_STR = "alert_front";
    public static final String RESP_ERROR_STR = "error";
    public static final int RESP_DONE_SZ = RESP_DONE_STR.length();
    public static final int RESP_OUT_OF_RANGE_SZ = RESP_OUT_OF_RANGE_STR.length();
    public static final int RESP_UNKNOWN_COMMAND_SZ = RESP_UNKNOWN_COMMAND_STR.length();
    public static final int RESP_ALERT_LEFT_SZ = RESP_ALERT_LEFT_STR.length();
    public static final int RESP_ALERT_RIGHT_SZ = RESP_ALERT_RIGHT_STR.length();
    public static final int RESP_ALERT_BACK_SZ = RESP_ALERT_BACK_STR.length();
    public static final int RESP_ALERT_FRONT_SZ = RESP_ALERT_FRONT_STR.length();
    public static final int RESP_ERROR_SZ = RESP_ERROR_STR.length();
    public static final int RESP_DONE = 0;
    public static final int RESP_OUT_OF_RANGE = 1;
    public static final int RESP_UNKNOWN_COMMAND = 2;
    public static final int RESP_ALERT_LEFT = 3;
    public static final int RESP_ALERT_RIGHT = 4;
    public static final int RESP_ALERT_BACK = 5;
    public static final int RESP_ALERT_FRONT = 6;
    public static final int RESP_ERROR = 7;
    public static final int RESP_TIMEOUT = -1;
    public static final int EYE_LEFT_RGB = 1;
    public static final int EYE_RIGHT_RGB = 2;
    public static final int EYE_DISTANCE_MAP = 3;
    private int timeout = 1000;
    private String hostname = "localhost";
    private int port = 5000;
    Socket socket = new Socket();

    private int readChar(InputStream is) throws IOException {
        long startTime = System.currentTimeMillis();
        while (is.available() == 0) {
            if ((System.currentTimeMillis() - startTime) > timeout) {
                throw new IOException("response timeouted");
            }
        }
        return is.read();
    }

    private int touchChar(InputStream is) throws IOException {
        long startTime = System.currentTimeMillis();
        while (is.available() == 0) {
            if ((System.currentTimeMillis() - startTime) > timeout) {
                throw new IOException("response timeouted");
            }
        }
        is.mark(1);
        int c = is.read();
        is.reset();
        return c;
    }

    private boolean isTokenChar(int c) {
        return Character.isJavaIdentifierPart(c) ||
                c == '.' || c == '+' || c == '-';

    }
    
    private String readToken(InputStream is) throws IOException {
        int c;
        String s = "";
        // vynech mezery
        do {
            c = readChar(is);
        } while (c == ' ' || c == '\r' || c == '\n' || c == '\t');

        s += (char) c;
        if (!isTokenChar(c)) {
            return s;
        }

        c = touchChar(is);
        // precti token
        while (isTokenChar(c)) {
            s += (char) c;
            readChar(is);
            c = touchChar(is);
        }
        // return token
        return s;
    }

    private byte[] readData(int size, InputStream is) throws IOException {
        int count = 0;
        byte[] data = new byte[size];
        do {
            long startTime = System.currentTimeMillis();
            int avail = is.available();
            count += is.read(data, count, Math.min(avail, size - count));
            if ((System.currentTimeMillis() - startTime) > timeout) {
//                throw new IOException("response timeouted");
            }
        } while (count < size);
        return data;
    }

    private int readCmdAck(InputStream is) throws IOException {
        int sz = 0;
        String s = "";
        long startTime = System.currentTimeMillis();
        do {
            int len = is.available();
            if (len > 0) {
                int c = is.read();
                if (isTokenChar(c)) {
                    s = "" + (char)c;
                    sz = 1;
                    break;
                }
            }

            if ((System.currentTimeMillis() - startTime) > timeout) {
                throw new IOException("response timeouted");
            }
        } while (true);
        
        do {
            int len = is.available();
            if (len > 0) {
                s += (char) is.read();
                sz++;
                if (sz == RESP_DONE_SZ && s.equals(RESP_DONE_STR)) {
                    return RESP_DONE;
                }
                if (sz == RESP_OUT_OF_RANGE_SZ && s.equals(RESP_OUT_OF_RANGE_STR)) {
                    return RESP_OUT_OF_RANGE;
                }
                if (sz == RESP_UNKNOWN_COMMAND_SZ && s.equals(RESP_UNKNOWN_COMMAND_STR)) {
                    return RESP_UNKNOWN_COMMAND;
                }
                if (sz == RESP_ALERT_LEFT_SZ && s.equals(RESP_ALERT_LEFT_STR)) {
                    return RESP_ALERT_LEFT;
                }
                if (sz == RESP_ALERT_RIGHT_SZ && s.equals(RESP_ALERT_RIGHT_STR)) {
                    return RESP_ALERT_RIGHT;
                }
                if (sz == RESP_ALERT_BACK_SZ && s.equals(RESP_ALERT_BACK_STR)) {
                    return RESP_ALERT_BACK;
                }
                if (sz == RESP_ALERT_FRONT_SZ && s.equals(RESP_ALERT_FRONT_STR)) {
                    return RESP_ALERT_FRONT;
                }
                if (sz == RESP_ERROR_SZ && s.equals(RESP_ERROR_STR)) {
                    return RESP_ERROR;
                }
            }
        } while ((System.currentTimeMillis() - startTime) < timeout);
        throw new IOException("response timeouted");
    }

    private int sendCommand(String cmd) throws IOException {
        String s;
        socket.getOutputStream().write(cmd.getBytes("ISO8859-1"));
        InputStream is = socket.getInputStream();
        return readCmdAck(is);
    }

    /** Konstruktor 
     * 
     * @param hostname IP adresa robota, na ktere robot nasloucha
     * @param port port, na kterem robot nasloucha
     * @throws java.io.IOException nastane v pripade, ze nedojde k navazani 
     * spojeni s robotem
     */
    public HardwareRobot(String hostname, int port) throws IOException {
        this.hostname = hostname;
        this.port = port;
    }

    /**
     * Tato metoda vytvori spojeni s robotem
     */
    public void connect() throws IOException {
        socket.connect(new InetSocketAddress(hostname, port));
    }

    /**
     * Tato metoda uzavre spojeni s robotem
     */
    public void disconnect() throws IOException {
        socket.close();
        socket = new Socket();
    }

    /** Tato metoda nastavuje maximalni rychlost robota v cm/s. Tato funkce
     * svazuje relativni hodnotu rychosti (pouzitou ve funkci setWheelSpeed)
     * s realnou rychlosti v cm/s. Volani teto metody se v aplikaci vyskytne
     * pouze jednou, pokud vicenasobne tak ve vemi omezene mire.
     *
     * Pokud speed je zaporne, robot omezeni maximalni rychlosti ingnoruje.
     *
     */
    public void setMaxWheelSpeed(double speed) throws IOException {
        int resp;
        String cmd;
        cmd = String.format(Locale.US,
                "set_max_wheel_speed %.2f;", speed);
        resp = sendCommand(cmd);
        if (resp != RESP_DONE) {
            throw new IOException("BlueCar command Error");
        }
    }

    /**
     *  Tato metoda nastavuje rychlost robota pro obe kola najednou. K navratu z
     * metody nedojde drive, dokud neni nastaveni potvrzeno robotem.
     *
     * @param left rychlost leveho kola. Cislo v rozsahu (-1,1). Zaporne cislo
     * znamena vzad, kladne cislo znamena vpred. Pokud je absolutni hodnota
     * rovna jedne, znamena to maximalni rychlost.
     * @param right rychlost praveho kola. Cislo v rozsahu (-1,1). Zaporne cislo
     * znamena vzad, kladne cislo znamena vpred. Pokud je absolutni hodnota
     * rovna jedne, znamena to maximalni rychlost.
     *
     * Pokud byla nastavena maximalni rychlost (setMaxWheelSpeed) na zapornou
     * hodnotu, pak hodnota parametru left (nebo right) ovlivnuje vykon
     * motoru v rozsahu 0-100% bez ohledu na dosazenou rychlost a
     * jeji mozne zmeny vlivem ruzneho napeti baterie.
     *
     * Pokud byla nastavena maximalni rychlost na kladnou hodnotu, pak se
     * predpoklada linearni zavislost mezi parametrem left (nebo right) a
     * rychlosti robota. Toto zajistuje robot bud zpetnou vazbou z kol, nebo
     * na zaklade tabulek. Chovani robota by (az na limitni pripady) nemelo
     * byt zavisle na napeti baterie.
     *
     */
    public void setWheelSpeed(double left, double right, double duration)
            throws IOException {
        int resp;
        String cmd;
        cmd = String.format(Locale.US,
                "set_wheel_speed %.1f %.1f %.3f;",
                left, right, duration);
        resp = sendCommand(cmd);
        if (resp != RESP_DONE
                && resp != RESP_ALERT_LEFT
                && resp != RESP_ALERT_RIGHT
                && resp != RESP_ALERT_BACK
                && resp != RESP_ALERT_FRONT) {
            throw new IOException("BlueCar command Error");
        }
    }

    //dhonza pouze docasne, duration bude nastavovano jinak
    public void setWheelSpeed(double left, double right) {
        try {
            setWheelSpeed(left, right, 5.0);
        } catch (IOException ex) {
            ex.printStackTrace();
        }
    }

    /**
     * 
     */
    public EyeImage getEyeImage(String what, boolean capture) throws IOException {
        int w, h, size;
        String s;
        String cmd = String.format(Locale.US, "get_eye_image %s %d;", what,
                (capture) ? 1 : 0);
        socket.getOutputStream().write(cmd.getBytes("ISO8859-1"));
        s = "";
       
        InputStream is = new BufferedInputStream(socket.getInputStream());
        // image
        s = readToken(is);
        if (!s.equals("image")) {
            throw new IOException("invalid response format");
        }
        // width
        s = readToken(is);
        w = Integer.parseInt(s);
        // height
        s = readToken(is);
        h = Integer.parseInt(s);
        // image type
        String imageType = readToken(is);
        // #
        s = readToken(is);
        if (!s.equals("#")) {
            throw new IOException("invalid response format");
        }
        // len
        s = readToken(is);
        try {
            size = Integer.parseInt(s);
        } catch (Exception ex) {
            throw new IOException("invalid response format");
        }
        // @
        s = readToken(is);
        if (!s.equals("@")) {
            throw new IOException("invalid response format");
        }
        // data
        byte[] data = readData(size, is);
        // @
        s = readToken(is);
        if (!s.equals("@")) {
            throw new IOException("invalid response format");
        }

        int ack = readCmdAck(is);
        if (ack != RESP_DONE) {
            throw new IOException("error reading image");
        }

        EyeImage image = EyeImage.createEyeImage(w, h, imageType, data);

        return image;

    }

    public Map<String,Object> getStatus() throws IOException {
        String cmd, s;
        double v;
        Map<String,Object> status = new HashMap<String,Object>();
        
        cmd = String.format(Locale.US,
                "get_status;");
        socket.getOutputStream().write(cmd.getBytes("ISO8859-1"));

        InputStream is = new BufferedInputStream(socket.getInputStream());

        s = readToken(is);
        if (!s.equals("{")) {
           throw new IOException("invalid response format");
        }

        // battery voltage
        s = readToken(is);
        v = Double.parseDouble(s);
        s = readToken(is);
        if (!s.equals(",")) {
           throw new IOException("invalid response format");
        }
        status.put("Battery voltage", v);

        // battery current
        s = readToken(is);
        v = Double.parseDouble(s);
        s = readToken(is);
        if (!s.equals(",")) {
           throw new IOException("invalid response format");
        }
        status.put("Battery current", v);

        s = readToken(is);
        v = Double.parseDouble(s);
        s = readToken(is);
        if (!s.equals(",")) {
           throw new IOException("invalid response format");
        }
        status.put("IR front-left sensor", v);

        s = readToken(is);
        v = Double.parseDouble(s);
        s = readToken(is);
        if (!s.equals(",")) {
           throw new IOException("invalid response format");
        }
        status.put("IR fron-right sensor", v);

        s = readToken(is);
        v = Double.parseDouble(s);
        status.put("IR back-center sensor", v);

        s = readToken(is);
        if (!s.equals("}")) {
           throw new IOException("invalid response format");
        }

        int ack = readCmdAck(is);
        if (ack != RESP_DONE) {
            throw new IOException("error reading status");
        }

        return status;
    }

    public String getHostname() {
        return hostname;
    }

    public int getPort() {
        return port;
    }

    public boolean isConnected() {
        return socket.isConnected();
    }

    public double[] getSensorData() {
        throw new IllegalStateException("Not yet implemented!");
    }

    @Override
    public void finalize() {
        try {
            System.out.println("Closing socket");
            socket.close();
        } catch (IOException ex) {
            Logger.getLogger(HardwareRobot.class.getName()).log(Level.SEVERE, null, ex);
        }
    }
}
    
