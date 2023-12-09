// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Util;
import frc.robot.constants.GameConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterConstants.ShooterState;

abstract class ShooterBase extends SubsystemBase {


  /** Creates a new ShooterBase. */
  protected static final TalonFX mFlyWheelMotor = new TalonFX(ShooterConstants.FLYWHEEL_TALON_ID,"rio");
  protected static final TalonFX mRollerMotor = new TalonFX(ShooterConstants.ROLLER_TALON_ID,"rio");

  protected double mRollerSetpoint = ShooterConstants.ShooterState.HOME.mRollerRPM;
  protected double mFlywheelSetpoint = ShooterConstants.ShooterState.HOME.mFlyWheelRPM;

  public final FlywheelSim mFlywheelSim = new FlywheelSim(DCMotor.getFalcon500(1),ShooterType.FLYWHEEL.mGearRatioToSensor,ShooterConstants.FLYWHEEL_MOI_KG_MPS,null);
  public final FlywheelSim mRollerSim = new FlywheelSim(DCMotor.getFalcon500(1),1/ShooterType.ROLLER.mGearRatioToSensor, ShooterConstants.ROLLER_MOI_KG_MPS,null);

  protected final TalonFXSimState mRollerFalconSim = mRollerMotor.getSimState();
  protected final TalonFXSimState mFlywheelFalconSim = mFlyWheelMotor.getSimState();

  protected final TalonFXConfiguration mRollerConfig = new TalonFXConfiguration();
  protected final TalonFXConfiguration mFlywheelConfig = new TalonFXConfiguration();

  protected final TalonFXConfigurator mFlywheelConfigurator = mFlyWheelMotor.getConfigurator();
  protected final TalonFXConfigurator mRollerConfigurator = mRollerMotor.getConfigurator();

  protected enum ShooterType{
    ROLLER(ShooterConstants.MOTOR_TO_ROLLER_GEAR_RATIO, mRollerMotor),
    FLYWHEEL(ShooterConstants.MOTOR_TO_FLWHEEL_GEAR_RATIO, mFlyWheelMotor);

    protected double mGearRatioToMotor;
    protected double mGearRatioToSensor;
    protected TalonFX mShooterObject;
    private ShooterType(double gearRatioToMotor, TalonFX shooterObject){
        mGearRatioToMotor = gearRatioToMotor;
        mGearRatioToSensor = 1/mGearRatioToMotor;
        mShooterObject = shooterObject;
    }
}

  protected TalonControlType mControlSignal = TalonControlType.VELOCITY_VOLTAGE;
  
  protected final VelocityVoltage mVelocityVoltage = new VelocityVoltage(0)
  .withSlot(0);

  protected final VelocityDutyCycle mDutyCycle = new VelocityDutyCycle(0)
  .withSlot(0);

  protected final CoastOut mCoastOut = new CoastOut()
  .withUpdateFreqHz(100);

  protected final VoltageOut mVoltageOut = new VoltageOut(0)
  .withUpdateFreqHz(100);

  protected ShooterBase() {
    mRollerMotor.getConfigurator().apply(new TalonFXConfiguration());
    mFlyWheelMotor.getConfigurator().apply(new TalonFXConfiguration());

    setOnboardFeedbackConstants();

    mFlyWheelMotor.setInverted(ShooterConstants.INVERT_FLYWHEEL_MOTOR);
    mRollerMotor.setInverted(ShooterConstants.INVERT_ROLLER_MOTOR);

    mRollerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    mFlywheelConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
  
  }
  protected abstract void setOnboardFeedbackConstants();

  abstract void runSpeedControl();
  /**
   * Set the desired velocity of the roller in nRPM
   * @return
   */
  abstract void setDesiredFlywheelRPM(double rpm);

  /**
   * Set the desired velocity of the roller in nRPM
   * @return
   */
  abstract void setDesiredRollerRPM(double rpm);

  abstract void changeFlywheelRPM(double increment);

  abstract boolean atDesiredFlywheelRPM();

  abstract boolean atDesiredRollerRPM();
  
  /**
   * Set the velocity of the flywheel MOTOR in native units (Rotations/Sec)
   * @return
   */
  public abstract void setFlywheelVelRaw(double rps);

  /**
   * Set the velocity of the roller MOTOR in native units (Rotations/Sec)
   * @return
   */
  public abstract void setRollerVelRaw(double rps);

  public void setMotorRaw(ShooterType mType, double rpm){
    switch(mControlSignal){
      case VELOCITY_VOLTAGE:
        mType.mShooterObject.setControl(mVelocityVoltage.withVelocity(velocityToNativeUnits(rpm, mType)));
        break;
      case DUTY_CYCLE_VEL:
        mType.mShooterObject.setControl(mDutyCycle.withVelocity(velocityToNativeUnits(rpm, mType)));
        break;
      case COAST_OUT:
        mType.mShooterObject.setControl(mCoastOut);
        break;
      default:
        mType.mShooterObject.setControl(mVelocityVoltage.withVelocity(velocityToNativeUnits(rpm, mType)));
        break;
    }
  }

  public void setToShooterState(ShooterState state){
    mControlSignal = TalonControlType.VELOCITY_VOLTAGE;
    setDesiredFlywheelRPM(state.mFlyWheelRPM);
    setDesiredRollerRPM(state.mRollerRPM);
  }

  public double getRollerRPM(){
    return nativeUnitsToVelocity(mRollerMotor.getRotorVelocity().getValue(),ShooterType.ROLLER);
  }
  /**
   * @return roller motor velocity in rotations per second of the MOTOR without gear ratio
   */
  public double getRollerVelRaw(){
    return mRollerMotor.getRotorVelocity().getValue();
  }

  public double getFlywheelVelRaw(){
    return mFlyWheelMotor.getRotorVelocity().getValue();
  }

  public double getFlywheelRPM(){
    return nativeUnitsToVelocity(mFlyWheelMotor.getRotorVelocity().getValue(),ShooterType.FLYWHEEL);
  }
  public void writeMotorDebugData(){
    SmartDashboard.putString("Exit Pose shooter", calcExitPos().toString());
    SmartDashboard.putNumber("Exit Angle shooter", calcExitAngle());
    SmartDashboard.putNumber("Roller RPS NativeUnits", mRollerMotor.getRotorVelocity().getValue());
    SmartDashboard.putNumber("FlyWheel RPS NativeUnits", mFlyWheelMotor.getRotorVelocity().getValue());

    SmartDashboard.putNumber("FlyWheel RPM", getFlywheelRPM());
    SmartDashboard.putNumber("RollerWheel RPM", getRollerRPM());
    
    SmartDashboard.putNumber("Flywheel Setpoint RPM", mFlywheelSetpoint);
    SmartDashboard.putNumber("Roller Setpoint RPM", mRollerSetpoint);

    SmartDashboard.putNumber("Native PID Output", mFlyWheelMotor.getClosedLoopOutput().getValue());
    SmartDashboard.putNumber("Native PID Error", mFlyWheelMotor.getClosedLoopError().getValue());
    SmartDashboard.putNumber("Native PID Setpoint", mFlyWheelMotor.getClosedLoopReference().getValue());

    SmartDashboard.putNumber("MOI",ShooterConstants.FLYWHEEL_MOI_KG_MPS);
    SmartDashboard.putNumber("Mass",ShooterConstants.FLYWHEEL_WEIGHT_KG);

    if (RobotBase.isSimulation()){
      SmartDashboard.putNumber("Flywheel sim output", mFlywheelSim.getAngularVelocityRPM());
      SmartDashboard.putNumber("Roller sim output", mRollerSim.getAngularVelocityRPM());
    }

    SmartDashboard.updateValues();
  }
  abstract void writeControllerDebugData();
  
  public void stopMotors(){
    mControlSignal = TalonControlType.COAST_OUT;
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

  protected double velocityToNativeUnits(double velocityRPM, ShooterType mode){
    double motorRotationsPerMin = velocityRPM * mode.mGearRatioToSensor;
    return motorRotationsPerMin/60;
  }

  protected double nativeUnitsToVelocity(double rotationsPerSecond, ShooterType mode){
    double motorRotationsPerMin = rotationsPerSecond * 60;
    return motorRotationsPerMin * mode.mGearRatioToMotor;
  }

  private static Translation3d calcExitPos(){
    Translation3d originTranslation = ShooterConstants.ROLLER_LOC_M.minus(ShooterConstants.FLYWHEEL_LOC_M);
    double midpoint = (originTranslation.getNorm()-ShooterConstants.ROLLER_RADIUS_M-ShooterConstants.FLYWHEEL_RADIUS_M)/2;
    Translation3d midPointVector = Util.getNormalTranslation(originTranslation).times((ShooterConstants.FLYWHEEL_RADIUS_M + midpoint));
    return midPointVector.plus(ShooterConstants.FLYWHEEL_LOC_M);
  }

  private static double calcExitAngle(){
    Translation3d originTranslation = ShooterConstants.ROLLER_LOC_M.minus(ShooterConstants.FLYWHEEL_LOC_M);
    Translation3d perpendicularVec = originTranslation.rotateBy(new Rotation3d(0, Math.PI/2, 0));
    Translation3d homePerpendicularVec = perpendicularVec.minus(originTranslation);
    return Math.toDegrees(Math.atan(homePerpendicularVec.getZ()/homePerpendicularVec.getX()));
  }

  private double calcExitVel(){
    //TODO
    return 2;
  }

  @Override
  public void simulationPeriodic(){
    writeMotorDebugData();

    mFlywheelSim.setInputVoltage(mFlywheelFalconSim.getMotorVoltage());
    mRollerSim.setInputVoltage(mRollerFalconSim.getMotorVoltage());

    mFlywheelSim.update(0.02);
    mRollerSim.update(0.02);

    mFlywheelFalconSim.setRotorVelocity(velocityToNativeUnits(mFlywheelSim.getAngularVelocityRPM(),ShooterType.FLYWHEEL));
    mRollerFalconSim.setRotorVelocity(velocityToNativeUnits(mRollerSim.getAngularVelocityRPM(),ShooterType.ROLLER));
  }
}
