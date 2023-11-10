// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Time;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class SubsystemCharacterization extends CommandBase {
  /** Creates a new SubsystemCharacterization. */
  private final DoubleSupplier mMeasurementSupplier;
  private final DoubleConsumer mOutputConsumer;
  private final double mMaxInput;
  private final double mSamplePeriodSec;
  private final Runnable mStopSystem;
  private double mPrevOutputVal;
  private double mPrevOutputTime;
  private double mMaxOutput;
  private double mMaxRate;
  private final Timer mTimer = new Timer();
  public SubsystemCharacterization(Subsystem plant, DoubleSupplier measurementSupplier,
  DoubleConsumer inputConsumer, Runnable stopSystem, double samplePeriodSec, double maxInput){
    // Use addRequirements() here to declare subsystem dependencies.
    mStopSystem = stopSystem;
    mMeasurementSupplier = measurementSupplier;
    mOutputConsumer = inputConsumer;
    mMaxInput = maxInput;
    mSamplePeriodSec = samplePeriodSec;
    addRequirements(plant);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mTimer.reset();
    mTimer.start();
    mMaxOutput = mMeasurementSupplier.getAsDouble();
    mPrevOutputVal = mMaxInput;
    mPrevOutputTime = 0;
    mMaxRate = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mOutputConsumer.accept(mMaxInput);
    double measurement = mMeasurementSupplier.getAsDouble();
    double rate = (measurement-mPrevOutputVal)/(mTimer.get()-mPrevOutputTime);
    if(measurement>mMaxOutput){mMaxOutput = measurement;}
    if(rate>mMaxRate){mMaxRate = measurement-mPrevOutputVal;}

    mPrevOutputTime = mTimer.get();
    mPrevOutputVal = measurement;

    SmartDashboard.putNumber("Plant Measurement", measurement);
    SmartDashboard.putNumber("Plant Rate", rate);
    SmartDashboard.putNumber("Plant Max Measurement", mMaxOutput);
    SmartDashboard.putNumber("Plant Max Rate",mMaxRate);
    SmartDashboard.updateValues();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mStopSystem.run();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mTimer.hasElapsed(mSamplePeriodSec);
  }
}
