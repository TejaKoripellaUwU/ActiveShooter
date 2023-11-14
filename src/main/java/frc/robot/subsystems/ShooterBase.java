// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util;
import frc.robot.constants.GameConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterConstants.ShooterState;

abstract class ShooterBase extends SubsystemBase {
  /** Creates a new ShooterBase. */
  protected final TalonFX mFlyWheelMotor = new TalonFX(ShooterConstants.FLYWHEEL_TALON_ID);
  protected final TalonFX mRollerMotor = new TalonFX(ShooterConstants.ROLLER_TALON_ID);

  protected double mRollerSetpoint = ShooterConstants.ShooterState.HOME.mRollerRPM;
  protected double mFlywheelSetpoint = ShooterConstants.ShooterState.HOME.mFlyWheelRPM;

  protected final TalonFXConfiguration mRollerConfig = new TalonFXConfiguration();
  protected final TalonFXConfiguration mFlywheelConfig = new TalonFXConfiguration();

  protected ShooterBase() {
    mFlyWheelMotor.configFactoryDefault();
    mRollerMotor.configFactoryDefault();

    setOnboardFeedbackConstants();

    mFlyWheelMotor.setInverted(ShooterConstants.INVERT_FLYWHEEL_MOTOR);
    mRollerMotor.setInverted(ShooterConstants.INVERT_ROLLER_MOTOR);

    mFlyWheelMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    mRollerMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    
    mFlyWheelMotor.setSensorPhase(ShooterConstants.FLYWHEEL_SENSOR_PHASE);
    mRollerMotor.setSensorPhase(ShooterConstants.ROLLER_SENSOR_PHASE);

    mFlyWheelMotor.configVoltageCompSaturation(12.0);
    mRollerMotor.configVoltageCompSaturation(12.0);

    mFlyWheelMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    mRollerMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);

    mFlyWheelMotor.enableVoltageCompensation(true);
    mRollerMotor.enableVoltageCompensation(true);
    
    mFlyWheelMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    mRollerMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
  }
  protected abstract void setOnboardFeedbackConstants();

  abstract void runSpeedControl();

  abstract void setDesiredFlywheelRPM(double rpm);

  abstract void setDesiredRollerRPM(double rpm);

  abstract void changeFlywheelRPM(double increment);

  abstract boolean atDesiredFlywheelRPM();

  abstract boolean atDesiredRollerRPM();

  public void setToShooterState(ShooterState state){
    setDesiredFlywheelRPM(state.mFlyWheelRPM);
    setDesiredRollerRPM(state.mRollerRPM);
  }

  public double getRollerRPM(){
    return mFlyWheelMotor.getSelectedSensorVelocity()*ShooterConstants.ENCODER_VEL_TO_ROLLER_RPM;
  }

  public double getFlywheelRPM(){
    return mRollerMotor.getSelectedSensorVelocity()*ShooterConstants.ENCODER_VEL_TO_FLYWHEEL_RPM;
  }
  public void writeMotorDebugData(){
    SmartDashboard.putNumber("RollerRPMNativeUnits", mFlyWheelMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("FlyWheelRPMNativeUnits", mRollerMotor.getSelectedSensorVelocity());

    SmartDashboard.putNumber("FlyWheelRPMReal", mFlyWheelMotor.getSelectedSensorVelocity()*ShooterConstants.ENCODER_VEL_TO_ROLLER_RPM);
    SmartDashboard.putNumber("RollerWheelRPMReal", mRollerMotor.getSelectedSensorVelocity()*ShooterConstants.ENCODER_VEL_TO_FLYWHEEL_RPM);
    
    SmartDashboard.putNumber("Flywheel Setpoint RPM", mFlywheelSetpoint);
    SmartDashboard.putNumber("Rol Setpoint RPM", mFlywheelSetpoint);
    SmartDashboard.updateValues();
  }
  abstract void writeControllerDebugData();
  
  public void stopMotors(){
    mFlyWheelMotor.set(TalonFXControlMode.Disabled, 0);
    mRollerMotor.set(TalonFXControlMode.Disabled,0);
  }

  private double getExitAngle(){
    /*
     double midPointMag = ShooterConstants.ROLLER_LOC_M.minus(ShooterConstants.FLYWHEEL_LOC_M).getNorm()-ShooterConstants.FLYWHEEL_RADIUS_CM/100-ShooterConstants.ROLLER_RADIUS_CM/100
    Translation3d midPointLoc = ShooterConstants.ROLLER_LOC_M.minus(ShooterConstants.FLYWHEEL_LOC_M)
    .minus(ShooterConstants.FLYWHEEL_RADIUS_CM+ShooterConstants.ROLLER_RADIUS_CM)
    .plus(ShooterConstants.FLYWHEEL_LOC_M);
    Math.asin(midPointLoc.getZ()/midPointLoc.getNorm())
    return ;
     */
    return 2;
  }

  private double getExitVelMPS(){
    return 2;
  }

  @Override
  public void periodic() {
    if(GameConstants.DEBUG_MODE){
      writeMotorDebugData();
      writeControllerDebugData();
    }
    runSpeedControl();
  }

  public double get2DMotion(double time){
    return getExitAngle()*getExitVelMPS();
  }
}
