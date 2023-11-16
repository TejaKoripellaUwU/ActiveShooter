// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sim;

/** Add your docs here. */
import frc.robot.sim.PhysicsSim.*;
import static frc.robot.sim.PhysicsSim.*; // random()

import com.ctre.phoenix.motorcontrol.can.*;

/**
 * Holds information about a simulated TalonSRX.
 */
class TalonFXSimProfile extends SimProfile {
    private final TalonFX mTalon;


    /** The current position */
    // private double _pos = 0;
    /** The current velocity */
    /**
     * Creates a new simulation profile for a TalonSRX device.
     * 
     * @param talon
     *        The TalonSRX device
     */
    public TalonFXSimProfile(final TalonFX talon) {
        mTalon = talon;
    }

    /**
     * Runs the simulation profile.
     * 
     * This uses very rudimentary physics simulation and exists to allow users to test
     * features of our products in simulation using our examples out of the box.
     * Users may modify this to utilize more accurate physics simulation.
     */
    public void run() {
        double outPerc = mTalon.getSimCollection().getMotorOutputLeadVoltage() / 12;
        mTalon.getSimCollection().setBusVoltage(12 - outPerc * outPerc * 3/4 * random(0.95, 1.05));
    }
}