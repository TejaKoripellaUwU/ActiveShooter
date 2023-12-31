// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Util.TBHController;
import frc.robot.constants.ShooterConstants;

public class ShooterTBH extends ShooterBase {
  /** Creates a new Shooter. */
  TBHController mFlywheelController = new TBHController(ShooterConstants.FLYWHEEL_TBH_CONSTANT, ShooterConstants.PHYSICAL_MAX_RPM_FLYWHEEL);
  TBHController mRollerController = new TBHController(ShooterConstants.ROLLER_TBH_CONSTANT, ShooterConstants.PHYSICAL_MAX_RPM_ROLLER);
  
  public ShooterTBH() {
    super();
  }
  protected void setOnboardFeedbackConstants(){
    //No onboard feedback necessary
  }

  public void setDesiredFlywheelRPM(double rpm){
    mControlSignal = TalonControlType.VOLTAGE_OUT;
    mFlywheelSetpoint = rpm;
    mFlywheelController.spinUp(mFlywheelSetpoint);
  }
  public void setDesiredRollerRPM(double rpm){
    mControlSignal = TalonControlType.VOLTAGE_OUT;
    mRollerSetpoint = rpm;
    mFlywheelController.spinUp(mRollerSetpoint);
  }
  public void setFlywheelVelRaw(double rps){
    mControlSignal = TalonControlType.VOLTAGE_OUT;
    mFlywheelSetpoint = nativeUnitsToVelocity(rps,ShooterType.FLYWHEEL);
    mFlywheelController.spinUp(mFlywheelSetpoint);
  }
   public void setRollerVelRaw(double rps){
    mControlSignal = TalonControlType.VOLTAGE_OUT;
    mRollerSetpoint = nativeUnitsToVelocity(rps,ShooterType.ROLLER);
    mRollerController.spinUp(mRollerSetpoint);
  }
  public void changeFlywheelRPM(double increment){
    mControlSignal = TalonControlType.VOLTAGE_OUT;
    mFlywheelSetpoint+=increment;
    mFlywheelController.spinUp(mFlywheelSetpoint);
  }
  public void changeRollerRPM(double increment){
    mControlSignal = TalonControlType.VOLTAGE_OUT;
    mRollerSetpoint+=increment;
    mRollerController.spinUp(mRollerSetpoint);
  }
  public boolean atDesiredFlywheelRPM(){
    return mFlywheelController.atSetpoint();
  }
  public boolean atDesiredRollerRPM(){
    return mFlywheelController.atSetpoint();
  }
  public void writeControllerDebugData(){
    SmartDashboard.putNumber("Flywheel Controller Output", mFlywheelController.getCurOutput());
    SmartDashboard.putNumber("Flywheel Controller Prev Error", mFlywheelController.getPrevError());
    SmartDashboard.putNumber("Flywheel Controller TBH Constant", mFlywheelController.getTBHConstant());

    SmartDashboard.putNumber("Roller Controller Output", mRollerController.getCurOutput());
    SmartDashboard.putNumber("Roller Controller Prev Error", mRollerController.getPrevError());
    SmartDashboard.putNumber("Roller Controller TBH Constant", mRollerController.getTBHConstant());
  }

  public void runSpeedControl(){
    switch(mControlSignal){
      case VELOCITY_VOLTAGE:
        break;
      case COAST_OUT:
        mFlyWheelMotor.setControl(mCoastOut);
        mRollerMotor.setControl(mCoastOut);
        break;
      case VOLTAGE_OUT:
        mFlyWheelMotor.setControl(mVoltageOut.withOutput(velocityToNativeUnits(mFlywheelController.calculate(mFlywheelSetpoint),ShooterType.FLYWHEEL)));
        mRollerMotor.setControl(mVoltageOut.withOutput(velocityToNativeUnits(mRollerController.calculate(mRollerSetpoint),ShooterType.ROLLER)));
        break;
      case DUTY_CYCLE_VEL:
        break;
    }
  }
}
