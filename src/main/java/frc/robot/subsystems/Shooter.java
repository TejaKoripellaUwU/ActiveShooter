// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import frc.robot.Util.TBHController;
import frc.robot.constants.ShooterConstants;

public class Shooter extends ShooterBase {
  /** Creates a new Shooter. */
  TBHController mFlywheelController = new TBHController(ShooterConstants.FLYWHEEL_TBH_CONSTANT, ShooterConstants.PHYSICAL_MAX_RPM_FLYWHEEL);
  TBHController mRollerController = new TBHController(ShooterConstants.ROLLER_TBH_CONSTANT, ShooterConstants.PHYSICAL_MAX_RPM_ROLLER);
  
  public Shooter() {
    super();
  }

  public void setDesiredFlywheelRPM(double rpm){
    mFlywheelSetpoint = rpm;
    mFlywheelController.spinUp(mFlywheelSetpoint);
  }
  public void setDesiredRollerRPM(double rpm){
    mRollerSetpoint = rpm;
    mFlywheelController.spinUp(mRollerSetpoint);
  }
  public void changeFlywheelRPM(double increment){
    mFlywheelSetpoint+=increment;
    mFlywheelController.spinUp(mFlywheelSetpoint);
  }
  public void changeRollerRPM(double increment){
    mRollerSetpoint+=increment;
    mRollerController.spinUp(mRollerSetpoint);
  }

  public boolean atDesiredFlywheelRPM(){
    return mFlywheelController.atSetpoint();
  }
  public boolean atDesiredRollerRPM(){
    return mFlywheelController.atSetpoint();
  }

  public void runSpeedControl(){
    mFlyWheelMotor.set(TalonFXControlMode.PercentOutput, mFlywheelController.calculate(getFlywheelRPM()));
    mRollerMotor.set(TalonFXControlMode.PercentOutput, mRollerController.calculate(getRollerRPM()));
  }
  
  @Override
  public void periodic() {
    runSpeedControl();
  }
}
