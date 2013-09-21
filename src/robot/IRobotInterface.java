package robot;

/**
 * @author bukz1
 *
 */

public interface IRobotInterface {

//	public void initSensors(double[] positions);
	
	public void setWheelSpeed(double left, double right);
	
	public double[] getSensorData();
	
}
