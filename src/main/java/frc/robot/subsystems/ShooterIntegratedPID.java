// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import frc.robot.constants.ShooterConstants;

public class ShooterIntegratedPID extends ShooterBase {
  /** Creates a new Shooter. */
  public ShooterIntegratedPID() {
    super();
  }
  protected void setOnboardFeedbackConstants(){
    //All PID constants need to be tuned

    mRollerConfig.slot1.kP = 0.2;
    mRollerConfig.slot1.kI = 0.0;
    mRollerConfig.slot1.kD = 0.05;
    mRollerConfig.slot1.kF = 0.01;
    mRollerConfig.slot1.allowableClosedloopError = 10;

    mFlywheelConfig.slot1.kP = 0.2;
    mFlywheelConfig.slot1.kI = 0.0;
    mFlywheelConfig.slot1.kD = 0.05;
    mFlywheelConfig.slot1.kF = 0.01;
    mFlywheelConfig.slot1.allowableClosedloopError = 10;
   
    mFlyWheelMotor.configAllSettings(mFlywheelConfig);
    mRollerMotor.configAllSettings(mRollerConfig);
    mFlyWheelMotor.selectProfileSlot(1, 0);
    mRollerMotor.selectProfileSlot(1, 0);
  }
  public void setDesiredFlywheelRPM(double rpm){
    mFlywheelSetpoint = rpm;
  }
  public void setDesiredRollerRPM(double rpm){
    mRollerSetpoint = rpm;
   }
  public void changeFlywheelRPM(double increment){
    mFlywheelSetpoint+=increment;
  }
  public void changeRollerRPM(double increment){
    mRollerSetpoint+=increment;
  }
  public boolean atDesiredFlywheelRPM(){
    return Math.abs(Math.abs(mFlywheelSetpoint)-Math.abs(getFlywheelRPM())) > ShooterConstants.FLYWHEEL_SP_DEADZONE;
  }
  public boolean atDesiredRollerRPM(){
    return Math.abs(Math.abs(mRollerSetpoint)-Math.abs(getRollerRPM())) > ShooterConstants.FLYWHEEL_SP_DEADZONE;
  }
  public void runSpeedControl(){
    mFlyWheelMotor.set(TalonFXControlMode.Velocity, mFlywheelSetpoint/ShooterConstants.ENCODER_VEL_TO_FLYWHEEL_RPM);
    mRollerMotor.set(TalonFXControlMode.Velocity, mRollerSetpoint/ShooterConstants.ENCODER_VEL_TO_ROLLER_RPM);
  }
  public void writeControllerDebugData(){
    //since we have no controller no debug data is provided
  }
  
}
