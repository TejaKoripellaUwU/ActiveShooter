package frc.robot.sim;

import java.util.*;
import com.ctre.phoenix.motorcontrol.can.*;

/**
 * Manages physics simulation for CTRE products.
 */
public class PhysicsSim {
    private static final PhysicsSim sim = new PhysicsSim();

    /**
     * Gets the robot simulator instance.
     */
    public static PhysicsSim getInstance() {
        return sim;
    }

    /**
     * Adds a VictorSPX controller to the simulator.
     * 
     * @param victor
     *        The VictorSPX device
     */
    public void addTalonFX(TalonFX talon) {
        if (talon != null) {
            TalonFXSimProfile simVictor = new TalonFXSimProfile(talon);
            mSimProfiles.add(simVictor);
        }
    }

    /**
     * Runs the simulator:
     * - enable the robot
     * - simulate TalonSRX sensors
     */
    public void run() {
        // Simulate devices
        for (SimProfile simProfile : mSimProfiles) {
            simProfile.run();
        }
    }

    private final ArrayList<SimProfile> mSimProfiles = new ArrayList<SimProfile>();

    /* scales a random domain of [0, 2pi] to [min, max] while prioritizing the peaks */
    static double random(double min, double max) {
        return (max - min) / 2 * Math.sin(Math.IEEEremainder(Math.random(), 2 * 3.14159)) + (max + min) / 2;
    }
    static double random(double max) {
        return random(0, max);
    }

    
    /**
     * Holds information about a simulated device.
     */
    static class SimProfile {
        private long mLastTime;
        private boolean mRunning = false;

        /**
         * Runs the simulation profile.
         * Implemented by device-specific profiles.
         */
        public void run() {}

        /**
         * Returns the time since last call, in milliseconds.
         */
        protected double getPeriod() {
            // set the start time if not yet running
            if (!mRunning) {
                mLastTime = System.nanoTime();
                mRunning = true;
            }
            
            long now = System.nanoTime();
            final double period = (now - mLastTime) / 1000000.;
            mLastTime = now;

            return period;
        }
    }
}