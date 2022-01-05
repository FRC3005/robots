package frc.lib.util;

import java.util.HashMap;
import java.util.HashSet;

import edu.wpi.first.wpilibj.Timer;

public class RunLimiter {
    private static final HashSet<Runnable> m_once = new HashSet<>();
    
    private static final HashMap<Runnable, Double> m_rateLimit = new HashMap<>();

    /**
     * Run a function only the first time called.
     * 
     * @param fcn function to call
     */
    public static void once(Runnable fcn) {
        if (m_once.add(fcn)) {
            fcn.run();
        }
    }

    /**
     * Run a function at a specified rate. Calls faster than the time
     * will be discarded.
     * 
     * @param fcn function to call
     * 
     * @param minTimeSeconds rate to call the function in seconds
     */
    public static void atRate(Runnable fcn, double minTimeSeconds) {
        double time = Timer.getFPGATimestamp();
        Double prev = m_rateLimit.put(fcn, time);

        if (prev == null || time - prev >= minTimeSeconds) {
            fcn.run();
        }
    }
}
