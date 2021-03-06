package vivae.fitness;

import vivae.blovstom.TomArena;
import vivae.arena.parts.Active;
import java.util.Iterator;
import java.util.Vector;

/**
 * Created by IntelliJ IDEA.
 * User: koutnij
 * Date: Nov 9, 2009
 * Time: 9:18:42 PM
 * To change this template use File | Settings | File Templates.
 */
public class Age extends FitnessFunction {

    TomArena arena;

    public Age(TomArena arena) {
        this.arena = arena;
    }

    @Override
    public double getFitness() {
        double res = 0d;
        Vector<Active> actives = arena.getActives();
        for (Iterator<Active> it = actives.iterator(); it.hasNext();) {
            Active ag = it.next();
            res += ag.agemeter;
        }
        return res / actives.size() / arena.stepsDone;
    }
}
