package frc.robot.CustomUtil;

import java.lang.reflect.Array;
import java.util.LinkedList;
import java.util.function.Consumer;

import edu.wpi.first.math.Pair;
/*
    Custom class to analyze a stream of data within a specified timeframe in real-time, given the length of the timeframe along with the frequency of updates
    @author WilliamBruce
*/
public class Timeframe<N extends Number> {
    private LinkedList<N> frame;
    private int seconds, updatesPerSecond, maxLen;
    public Timeframe(int seconds, int updatesPerSecond) {
        frame = new LinkedList<>();
        this.seconds = seconds;
        this.updatesPerSecond = updatesPerSecond;
        maxLen = this.seconds * this.updatesPerSecond;
    }

    /*
        Updates the Timeframe class with a new input
        @param value the value to update the Timeframe with
    */
    public void update(N value) {
        if(frame.size() >= maxLen) { //If the current data does not take up a whole timeframe, then simply add this datapoint, else remove a point first
            frame.remove();
        }
        frame.add(value);
    }

    /*
        Get a percentage of existing entries that are greater than a target number
        @param target the target to compare other data points to
    */
    public double percentGreaterThan(N target, boolean includeEqual) {
        int matches = 0;
        for(int i = 0; i < frame.size(); i++) {
            if(includeEqual && frame.get(i).doubleValue() >= target.doubleValue()) {
                matches++;
            } else if(!includeEqual && frame.get(i).doubleValue() > target.doubleValue()) {
                matches++;
            }
        }
        return ((double)matches) / frame.size();
    }

    /*
        Get a percentage of existing entries that are less than a target number
        @param target the target to compare other data points to
    */
    public double percentLessThan(N target, boolean includeEqual) {
        return 1.0 - percentGreaterThan(target, includeEqual);
    }

    public void forEach(Consumer<? super N> action) {
        frame.forEach(action);
    }
}
