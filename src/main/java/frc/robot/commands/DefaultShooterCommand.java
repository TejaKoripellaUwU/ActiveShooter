// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Input;
import frc.robot.constants.InputConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterConstants.ShooterState;
import frc.robot.subsystems.ShooterIntegratedPID;

public class DefaultShooterCommand extends CommandBase {
  /** Creates a new DefaultShooterCommand. */
  boolean mToggleCoast = false;
  ShooterIntegratedPID mShooter;
  public DefaultShooterCommand(ShooterIntegratedPID shooter) {
    mShooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Input.getAPressed()){
      mShooter.setToShooterState(ShooterState.READY);
    }
    if (Input.getBPressed()){
      mToggleCoast = !mToggleCoast;
      if (mToggleCoast){
        mShooter.stopMotors();
      }
    }
    if (Input.getYPressed()){
      mShooter.setToShooterState(ShooterState.STATE_1);
    }
    if (Input.getXPressed()){
      mShooter.setToShooterState(ShooterState.STATE_2);
    }
    if (Math.abs(Input.getLeftJoyY())>InputConstants.LSTICKDEADZONE){
      mShooter.changeFlywheelRPM(Input.getLeftJoyY()*ShooterConstants.PHYSICAL_MAX_RPM_FLYWHEEL);
    }
    if (Math.abs(Input.getRightJoyY())>InputConstants.RSTICKDEADZONE){
      mShooter.changeFlywheelRPM(Input.getRightJoyY()*ShooterConstants.PHYSICAL_MAX_RPM_ROLLER);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
