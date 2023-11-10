// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DefaultShooterCommand;
import frc.robot.commands.SubsystemCharacterization;
import frc.robot.constants.GameConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.ShooterIntegratedPID;
import frc.robot.subsystems.ShooterTBH;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  ShooterIntegratedPID mShooter = new ShooterIntegratedPID();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if(GameConstants.RUN_SHOOTER_CHARACTERIZATION){
      mShooter.setDefaultCommand(new SubsystemCharacterization(mShooter, mShooter::getFlywheelRPM,
       (double rpm)->mShooter.setDesiredFlywheelRPM(rpm), mShooter::stopMotors,
        5, ShooterConstants.PHYSICAL_MAX_RPM_FLYWHEEL));
    } else{
      mShooter.setDefaultCommand(new DefaultShooterCommand(mShooter));
    }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
