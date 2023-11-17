// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.ejml.simple.SimpleMatrix;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
import edu.wpi.first.math.geometry.Rotation3d;
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

  protected TalonFXControlMode mMotorControl = TalonFXControlMode.Velocity;

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

  public void setMotorRaw(){
    mFlyWheelMotor.set(ControlMode.MotionMagic, mFlywheelSetpoint);
  }

  public void setToShooterState(ShooterState state){
    mMotorControl = TalonFXControlMode.Velocity;
    setDesiredFlywheelRPM(state.mFlyWheelRPM);
    setDesiredRollerRPM(state.mRollerRPM);
  }

  public double getRollerRPM(){
    return nativeUnitsToVelocity(mRollerMotor.getSelectedSensorVelocity(),ShooterType.ROLLER);
  }

  public double getFlywheelRPM(){
    return nativeUnitsToVelocity(mFlyWheelMotor.getSelectedSensorVelocity(),ShooterType.FLYWHEEL);
  }
  public void writeMotorDebugData(){
    SmartDashboard.putString("Exit Pose shooter", calcExitPos().toString());
    SmartDashboard.putNumber("Exit Angle shooter", calcExitAngle());
    SmartDashboard.putNumber("RollerRPMNativeUnits", mRollerMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("FlyWheelRPMNativeUnits", mFlyWheelMotor.getSelectedSensorVelocity());

    SmartDashboard.putNumber("FlyWheelRPMReal", getFlywheelRPM());
    SmartDashboard.putNumber("RollerWheelRPMReal", getRollerRPM());
    

    SmartDashboard.putNumber("Flywheel Setpoint RPM", mFlywheelSetpoint);
    SmartDashboard.putNumber("Roller Setpoint RPM", mRollerSetpoint);

    SmartDashboard.putNumber("PID output", mFlyWheelMotor.getClosedLoopTarget());
    SmartDashboard.putNumber("PID Error", mFlyWheelMotor.getClosedLoopError());

    
    SmartDashboard.updateValues();
   
  }
  abstract void writeControllerDebugData();
  
  public void stopMotors(){
    mMotorControl = TalonFXControlMode.Disabled;
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


  protected int velocityToNativeUnits(double velocityRPM, ShooterType mode){
    double motorRotationsPerMin = velocityRPM * mode.mGearRatioToSensor;
    double motorRotationsPer100ms = motorRotationsPerMin / 600;
    return (int)(motorRotationsPer100ms * ShooterConstants.ENCODER_TICKS_PER_ROTATION);
  }

  protected double nativeUnitsToVelocity(double sensorCounts, ShooterType mode){
    double motorRotationsPer100ms = sensorCounts / ShooterConstants.ENCODER_TICKS_PER_ROTATION;
    double motorRotationsPerMin = motorRotationsPer100ms * 600;
    return motorRotationsPerMin * mode.mGearRatioToMotor;
  }

  private Translation3d calcExitPos(){
    Translation3d originTranslation = ShooterConstants.ROLLER_LOC_M.minus(ShooterConstants.FLYWHEEL_LOC_M);
    double midpoint = (originTranslation.getNorm()-ShooterConstants.ROLLER_RADIUS_M-ShooterConstants.FLYWHEEL_RADIUS_M)/2;
    Translation3d midPointVector = Util.getNormalTranslation(originTranslation).times((ShooterConstants.FLYWHEEL_RADIUS_M + midpoint));
    return midPointVector.plus(ShooterConstants.FLYWHEEL_LOC_M);
  }

  private double calcExitAngle(){
    Translation3d originTranslation = ShooterConstants.ROLLER_LOC_M.minus(ShooterConstants.FLYWHEEL_LOC_M);
    Translation3d perpendicularVec = originTranslation.rotateBy(new Rotation3d(0, Math.PI/2, 0));
    Translation3d homePerpendicularVec = perpendicularVec.minus(originTranslation);
    return Math.toDegrees(Math.atan(homePerpendicularVec.getZ()/homePerpendicularVec.getX()));
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

    mFlywheelFalconSim.setIntegratedSensorVelocity(velocityToNativeUnits(mFlywheelSim.getAngularVelocityRPM(),ShooterType.FLYWHEEL));
    mRollerFalconSim.setIntegratedSensorVelocity(velocityToNativeUnits(mRollerSim.getAngularVelocityRPM(),ShooterType.ROLLER));
  }
}
