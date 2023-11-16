// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.ejml.simple.SimpleMatrix;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Input;
import frc.robot.Robot;
import frc.robot.Util;
import frc.robot.constants.GameConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterConstants.ShooterState;

abstract class ShooterBase extends SubsystemBase {
  /** Creates a new ShooterBase. */
  protected final WPI_TalonFX mFlyWheelMotor = new WPI_TalonFX(ShooterConstants.FLYWHEEL_TALON_ID,"rio");
  protected final WPI_TalonFX mRollerMotor = new WPI_TalonFX(ShooterConstants.ROLLER_TALON_ID,"rio");

  protected double mRollerSetpoint = ShooterConstants.ShooterState.HOME.mRollerRPM;
  protected double mFlywheelSetpoint = ShooterConstants.ShooterState.HOME.mFlyWheelRPM;

  public final FlywheelSim mFlywheelSim = new FlywheelSim(DCMotor.getFalcon500(1),1/ShooterConstants.MOTOR_TO_FLWHEEL_GEAR_RATIO, 0.00610837488,null);
  public final FlywheelSim mRollerSim = new FlywheelSim(DCMotor.getFalcon500(1),1/ShooterConstants.MOTOR_TO_FLWHEEL_GEAR_RATIO, 0.00610837488,null);

  protected final TalonFXSimCollection mRollerFalconSim = mRollerMotor.getSimCollection();
  protected final TalonFXSimCollection mFlywheelFalconSim = mFlyWheelMotor.getSimCollection();

  protected final TalonFXConfiguration mRollerConfig = new TalonFXConfiguration();
  protected final TalonFXConfiguration mFlywheelConfig = new TalonFXConfiguration();

  protected TalonFXControlMode motorControl = TalonFXControlMode.Velocity;

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
    motorControl = TalonFXControlMode.Velocity;
    setDesiredFlywheelRPM(state.mFlyWheelRPM);
    setDesiredRollerRPM(state.mRollerRPM);
  }

  public double getRollerRPM(){
    if(Robot.isSimulation()){
      return mRollerSim.getAngularVelocityRPM();
    }
    return mFlyWheelMotor.getSelectedSensorVelocity()*ShooterConstants.ENCODER_VEL_TO_ROLLER_RPM;
  }

  public double getFlywheelRPM(){
    if(Robot.isSimulation()){
      return mFlywheelSim.getAngularVelocityRPM();
    }
    return mRollerMotor.getSelectedSensorVelocity()*ShooterConstants.ENCODER_VEL_TO_FLYWHEEL_RPM;
  }
  public void writeMotorDebugData(){
    SmartDashboard.putNumber("RollerRPMNativeUnits", mRollerMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("FlyWheelRPMNativeUnits", mFlyWheelMotor.getSelectedSensorVelocity());

    SmartDashboard.putNumber("FlyWheelRPMReal", mFlyWheelMotor.getSelectedSensorVelocity()*ShooterConstants.ENCODER_VEL_TO_ROLLER_RPM);
    SmartDashboard.putNumber("RollerWheelRPMReal", mRollerMotor.getSelectedSensorVelocity()*ShooterConstants.ENCODER_VEL_TO_FLYWHEEL_RPM);
    
    SmartDashboard.putNumber("Flywheel Setpoint RPM", mFlywheelSetpoint);
    SmartDashboard.putNumber("Roller Setpoint RPM", mRollerSetpoint);

    SmartDashboard.putNumber("PID output", mFlyWheelMotor.getClosedLoopTarget());
    SmartDashboard.putNumber("PID Error", mFlyWheelMotor.getClosedLoopError());

    
    SmartDashboard.updateValues();
   
  }
  abstract void writeControllerDebugData();
  
  public void stopMotors(){
    motorControl = TalonFXControlMode.Disabled;
    mFlywheelSetpoint = 0;
    mRollerSetpoint = 0;
  }

  @Override
  public void periodic() {
    mRollerSetpoint = MathUtil.clamp(mRollerSetpoint, -ShooterConstants.PHYSICAL_MAX_RPM_ROLLER, ShooterConstants.PHYSICAL_MAX_RPM_ROLLER);
    mFlywheelSetpoint = MathUtil.clamp(mFlywheelSetpoint, -ShooterConstants.PHYSICAL_MAX_RPM_FLYWHEEL, ShooterConstants.PHYSICAL_MAX_RPM_FLYWHEEL);

    if(GameConstants.DEBUG_MODE){
      writeMotorDebugData();
      writeControllerDebugData();
    }
    runSpeedControl();
  }


  protected int velocityToNativeUnits(double velocityRPM){
    double motorRotationsPerMin = velocityRPM / ShooterConstants.MOTOR_TO_ROLLER_GEAR_RATIO;
    double motorRotationsPer100ms = motorRotationsPerMin / 600;
    int sensorCountsPer100ms = (int)(motorRotationsPer100ms * ShooterConstants.ENCODER_TICKS_PER_ROTATION);
    return sensorCountsPer100ms;
  }

  protected double nativeUnitsToVelocity(double sensorCounts){
    double motorRotationsPer100ms = (double)sensorCounts / ShooterConstants.ENCODER_TICKS_PER_ROTATION;
    double motorRotationsPerMin = motorRotationsPer100ms * 600;
    double flywheelRotationPerMin = motorRotationsPerMin * ShooterConstants.MOTOR_TO_FLWHEEL_GEAR_RATIO;
    return flywheelRotationPerMin;
  }

  @Override
  public void simulationPeriodic(){
    writeMotorDebugData();

    mFlywheelFalconSim.setBusVoltage(RobotController.getBatteryVoltage());
    mRollerFalconSim.setBusVoltage(RobotController.getBatteryVoltage());

    mFlywheelSim.setInputVoltage(mFlywheelFalconSim.getMotorOutputLeadVoltage());
    mRollerSim.setInputVoltage(mRollerFalconSim.getMotorOutputLeadVoltage());

    mFlywheelSim.update(0.02);
    mRollerSim.update(0.02);

    SmartDashboard.putNumber("Flywheel sim output", mFlywheelSim.getAngularVelocityRPM());

    mFlywheelFalconSim.setIntegratedSensorVelocity(velocityToNativeUnits(mFlywheelSim.getAngularVelocityRPM()));
    mRollerFalconSim.setIntegratedSensorVelocity(velocityToNativeUnits(mRollerSim.getAngularVelocityRPM()));
  }
}
