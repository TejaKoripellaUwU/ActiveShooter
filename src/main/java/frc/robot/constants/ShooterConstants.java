// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */
public class ShooterConstants {
    public enum ShooterState{
        READY(1000,1000),
        STATE_1(100,10),
        STATE_2(200,50),
        HOME(0,0);
        public final double mFlyWheelRPM;
        public final double mRollerRPM;
        ShooterState(double flyWheelRPM, double rollerRPM){
          mFlyWheelRPM = flyWheelRPM;
          mRollerRPM = rollerRPM;
        }
      }

    private static final double ENCODER_TICKS_PER_ROTATION = 2048;
    private static final double PHYSICAL_MAX_RPM_FALCON = 6380;

    //2048 sensor units/ 100 ms
    private static final double ENCODER_VEL_TO_MOTOR_RPM = 10*2*Math.PI/ENCODER_TICKS_PER_ROTATION; 
    private static final double MOTOR_TO_FLWHEEL_GEAR_RATIO = 5.0/3;
    private static final double MOTOR_TO_ROLLER_GEAR_RATIO = 5.0/3;

    public static final double ENCODER_VEL_TO_FLYWHEEL_RPM = ENCODER_VEL_TO_MOTOR_RPM * MOTOR_TO_FLWHEEL_GEAR_RATIO;
    public static final double ENCODER_VEL_TO_ROLLER_RPM = ENCODER_VEL_TO_MOTOR_RPM * MOTOR_TO_ROLLER_GEAR_RATIO;
    
    public static final double PHYSICAL_MAX_RPM_ROLLER= PHYSICAL_MAX_RPM_FALCON * MOTOR_TO_ROLLER_GEAR_RATIO;
    public static final double PHYSICAL_MAX_RPM_FLYWHEEL = PHYSICAL_MAX_RPM_FALCON * MOTOR_TO_FLWHEEL_GEAR_RATIO;

    public static final boolean INVERT_FLYWHEEL_MOTOR = false;//not done
    public static final boolean INVERT_ROLLER_MOTOR = true;//not done

    public static final double FLYWHEEL_TBH_CONSTANT = 0.03;
    public static final double ROLLER_TBH_CONSTANT = 0;

    public static final boolean FLYWHEEL_SENSOR_PHASE = false;
    public static final boolean ROLLER_SENSOR_PHASE = false;

    public static final Translation3d FLYWHEEL_LOC_M = new Translation3d(-0.5, 0, 0.4);
    public static final Translation3d ROLLER_LOC_M = new Translation3d(-0.8,0,0.5);

    public static final double FLYWHEEL_RADIUS_CM = 20;
    public static final double ROLLER_RADIUS_CM = 20;
    public static final double FLYWHEEL_SP_DEADZONE = 0;
    
    public static final int FLYWHEEL_TALON_ID = 8;
    public static final int ROLLER_TALON_ID = 7;

}
