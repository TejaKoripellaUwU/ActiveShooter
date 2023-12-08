// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;

import frc.robot.Util.PIDConstants;
import frc.robot.constants.ShooterConstants;

public class ShooterIntegratedPID extends ShooterBase {
  /** Creates a new Shooter. */
  public ShooterIntegratedPID() {
    super();
  }

  protected void setOnboardFeedbackConstants(){
    //All PID constants need to be tuned
    MotionMagicConfigs magicConfigs = new MotionMagicConfigs();
    magicConfigs.MotionMagicAcceleration = 108.33/3;
    magicConfigs.MotionMagicAcceleration = 2*108.33/1.5;
    mFlywheelConfigurator.apply(ShooterConstants.FLYWHEEL_PID.toTalonConfiguration());
    mRollerConfigurator.apply(ShooterConstants.ROLLER_PID.toTalonConfiguration());
  }
  public void setDesiredFlywheelRPM(double rpm){
    mControlSignal = TalonControlType.VELOCITY_VOLTAGE;
    mFlywheelSetpoint = rpm;
  }
  public void setDesiredRollerRPM(double rpm){
    mControlSignal = TalonControlType.VELOCITY_VOLTAGE;
    mRollerSetpoint = rpm;
   }
   public void setFlywheelVelRaw(double rps){
    mControlSignal = TalonControlType.VELOCITY_VOLTAGE;
    mFlywheelSetpoint = nativeUnitsToVelocity(rps,ShooterType.FLYWHEEL);
   }
   public void setRollerVelRaw(double rps){
    mControlSignal = TalonControlType.VELOCITY_VOLTAGE;
    mRollerSetpoint = nativeUnitsToVelocity(rps,ShooterType.ROLLER);
   }
  public void changeFlywheelRPM(double increment){
    mControlSignal = TalonControlType.VELOCITY_VOLTAGE;
    mFlywheelSetpoint+=increment;
  }
  public void changeRollerRPM(double increment){
    mControlSignal = TalonControlType.VELOCITY_VOLTAGE;
    mRollerSetpoint+=increment;
  }

  public boolean atDesiredFlywheelRPM(){
    return Math.abs(Math.abs(mFlywheelSetpoint)-Math.abs(getFlywheelRPM())) > ShooterConstants.FLYWHEEL_SP_DEADZONE;
  }
  public boolean atDesiredRollerRPM(){
    return Math.abs(Math.abs(mRollerSetpoint)-Math.abs(getRollerRPM())) > ShooterConstants.FLYWHEEL_SP_DEADZONE;
  }
  public void runSpeedControl(){
    switch(mControlSignal){
      case VELOCITY_VOLTAGE:
        mFlyWheelMotor.setControl(mVelocityVoltage.withVelocity(velocityToNativeUnits(mFlywheelSetpoint,ShooterType.FLYWHEEL)));
        mRollerMotor.setControl(mVelocityVoltage.withVelocity(velocityToNativeUnits(mRollerSetpoint,ShooterType.FLYWHEEL)));
        break;
      case COAST_OUT:
        mFlyWheelMotor.setControl(mCoastOut);
        mRollerMotor.setControl(mCoastOut);
        break;
      case DUTY_CYCLE_VEL:
        mFlyWheelMotor.setControl(mDutyCycle.withVelocity(velocityToNativeUnits(mFlywheelSetpoint,ShooterType.FLYWHEEL)));
        mRollerMotor.setControl(mDutyCycle.withVelocity(velocityToNativeUnits(mRollerSetpoint,ShooterType.FLYWHEEL)));
        break;
      case VOLTAGE_OUT:
        break;
    }
  }

  public void setTunablePIDFlywheel(PIDConstants pidFlywheel){
    mFlywheelConfigurator.apply(pidFlywheel.toTalonConfiguration());
  }

  public void setTunablePIDRoller(PIDConstants pidRoller){
    mRollerConfigurator.apply(pidRoller.toTalonConfiguration());
  }

  public void writeControllerDebugData(){
    //since we have no controller no debug data is provided
  }
  
}
