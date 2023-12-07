// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;
import frc.robot.constants.GameConstants;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Util.PIDConstants;

/** Add your docs here. */
public class ShooterConstants {
    public enum ShooterState{
        READY(1000,1000),
        STATE_1(2000,1000),
        STATE_2(4000,2000),
        HOME(0,0);
        public final double mFlyWheelRPM;
        public final double mRollerRPM;
        ShooterState(double flyWheelRPM, double rollerRPM){
          mFlyWheelRPM = flyWheelRPM;
          mRollerRPM = rollerRPM;
        }
      }

    public static final double ENCODER_TICKS_PER_ROTATION = 2048;
    public static final double PHYSICAL_MAX_RPM_FALCON = 6380;

    //2048 sensor units/ 100 ms  -> RPM
    public static final double ENCODER_VEL_TO_MOTOR_RPM = 600/ENCODER_TICKS_PER_ROTATION; 
    public static final double MOTOR_TO_FLWHEEL_GEAR_RATIO = 1;
    public static final double MOTOR_TO_ROLLER_GEAR_RATIO = 5.0/3;

    public static final double ENCODER_VEL_TO_FLYWHEEL_RPM = ENCODER_VEL_TO_MOTOR_RPM * MOTOR_TO_FLWHEEL_GEAR_RATIO;
    public static final double ENCODER_VEL_TO_ROLLER_RPM = ENCODER_VEL_TO_MOTOR_RPM * MOTOR_TO_ROLLER_GEAR_RATIO;
    
    public static final double PHYSICAL_MAX_RPM_ROLLER= PHYSICAL_MAX_RPM_FALCON * MOTOR_TO_ROLLER_GEAR_RATIO;
    public static final double PHYSICAL_MAX_RPM_FLYWHEEL = PHYSICAL_MAX_RPM_FALCON * MOTOR_TO_FLWHEEL_GEAR_RATIO;

    public static final double FLYWHEEL_MAX_ACCEL_ROTPS = 14703.66; //determined by simulated subsystem characterization

    public static final boolean INVERT_FLYWHEEL_MOTOR = false;//not done
    public static final boolean INVERT_ROLLER_MOTOR = true;//not done

    public static final double FLYWHEEL_TBH_CONSTANT = 0.03;
    public static final double ROLLER_TBH_CONSTANT = 0;

    public static final boolean FLYWHEEL_SENSOR_PHASE = false;
    public static final boolean ROLLER_SENSOR_PHASE = false;

    public static final Translation3d FLYWHEEL_LOC_M = new Translation3d(-0.5, 0, 0.4);
    public static final Translation3d ROLLER_LOC_M = new Translation3d(-0.8,0,0.5);

    public static final double FLYWHEEL_RADIUS_M = Units.inchesToMeters(3);
    public static final double ROLLER_RADIUS_M = Units.inchesToMeters(3);

    public static final double FLYWHEEL_WEIGHT_KG = Units.lbsToKilograms(0.58);
    public static final double ROLLER_WEIGHT_KG = Units.lbsToKilograms(0.58);

    public static final double FLYWHEEL_MOI_CONSTANT = 1.0/2;
    public static final double ROLLER_MOI_CONSTANT = 1.0/2;

    public static final double FLYWHEEL_MOI_KG_MPS = FLYWHEEL_MOI_CONSTANT*FLYWHEEL_WEIGHT_KG*Math.pow(FLYWHEEL_RADIUS_M,2);
    public static final double ROLLER_MOI_KG_MPS = ROLLER_MOI_CONSTANT*ROLLER_WEIGHT_KG*Math.pow(ROLLER_RADIUS_M,2);

    public static final double FLYWHEEL_SP_DEADZONE = 0.05;
    public static final double ROLLER_SP_DEADZONE = 0.05;
    
    public static final int FLYWHEEL_TALON_ID = 8;
    public static final int ROLLER_TALON_ID = 7;
    
    public static final PIDConstants FLYWHEEL_PID = 
    new PIDConstants(3,1.0,0.02,0,0.17);
  
    public static final PIDConstants ROLLER_PID = 
    new PIDConstants(3,1.0,0.02,0,0.17);
    public static PIDConstants TUNABLE_FLYWHEEL_PID = 
    new PIDConstants(3,1.0,0.02,0,0.17);  
    public static PIDConstants TUNABLE_ROLLER_PID = 
    new PIDConstants(3,1.0,0.02,0,0.17);}
