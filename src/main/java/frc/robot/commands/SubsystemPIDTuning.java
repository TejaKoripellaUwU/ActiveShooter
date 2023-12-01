// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Input;
import frc.robot.Util.PIDConstants;
import frc.robot.Util.POV;

public class SubsystemPIDTuning extends CommandBase {
  private final Consumer<PIDConstants> mPIDConsumer;
  private final DoubleConsumer mSetPointConsumer;
  private final DoubleSupplier mMeasurementSupplier;
  private PIDConstants mPID = new PIDConstants(0, 0, 0);
  private double mSetPoint = 0;
  private boolean mToggled = false;

  /** Creates a new PIDTuning. */
  public SubsystemPIDTuning(Consumer<PIDConstants> pidConsumer,
   DoubleConsumer setPointConsumer, DoubleSupplier measurementSupplier, Subsystem mSub) {
    mPIDConsumer = pidConsumer;
    mSetPointConsumer = setPointConsumer;
    mMeasurementSupplier = measurementSupplier;
    addRequirements(mSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("kP", 0);
    SmartDashboard.putNumber("kI", 0);
    SmartDashboard.putNumber("kD", 0);
    SmartDashboard.putNumber("kS", 0);
    SmartDashboard.putNumber("kV", 0);
    SmartDashboard.putNumber("increment", 2);
    SmartDashboard.putNumber("error",mSetPoint-mMeasurementSupplier.getAsDouble());
    SmartDashboard.putNumber("setpoint", mSetPoint);
    SmartDashboard.putBoolean("toggled", mToggled);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double increment = 0;

    if (Input.getLeftBumper()) {
      mToggled = !mToggled;
    }

    mToggled = SmartDashboard.getBoolean("toggled", mToggled);
    mPID.mKP = SmartDashboard.getNumber("kP", 0);
    mPID.mKI = SmartDashboard.getNumber("kI", 0);
    mPID.mKD = SmartDashboard.getNumber("kD", 0);
    mPID.mKS = SmartDashboard.getNumber("kS", 0);
    mPID.mKV = SmartDashboard.getNumber("kV", 0);
    increment = SmartDashboard.getNumber("increment", 2);

    mPIDConsumer.accept(mPID);

    if(mToggled){
      mSetPoint = SmartDashboard.getNumber("setpoint", 0);
      mSetPointConsumer.accept(mSetPoint);
    }
    
    if (Input.getPOV() == POV.DPADUP.mAngle) {
      mSetPoint += increment;
    } else if (Input.getPOV() == POV.DPADDOWN.mAngle) {
      mSetPoint -= increment;
    }

    SmartDashboard.putNumber("measurement", mMeasurementSupplier.getAsDouble());
    SmartDashboard.putNumber("error", mSetPoint-mMeasurementSupplier.getAsDouble());

    SmartDashboard.updateValues();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // For now we are keeping this empty to handle interruptions
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
