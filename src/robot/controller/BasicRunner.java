package robot.controller;

/**
 * Created by IntelliJ IDEA.
 * User: drchaj1
 * Date: Apr 28, 2010
 * Time: 11:05:57 AM
 * To change this template use File | Settings | File Templates.
 */
public class BasicRunner implements Runner {
    final private IRobotController controller;
    final private int steps;

    public BasicRunner(IRobotController controller, int steps) {
        this.controller = controller;
        this.steps = steps;
    }

    public void run() {
        for (int i = 0; i < steps; i++) {
            controller.step();            
        }
    }
}
