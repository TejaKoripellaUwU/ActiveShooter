// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.SubsystemCharacterization;
import frc.robot.commands.SubsystemPIDTuning;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.GameConstants;
import frc.robot.constants.ShooterConstants;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  


  /*
   * These numbers are an example AndyMark Drivetrain with some additional weight.  This is a fairly light robot.
   * Note you can utilize results from robot characterization instead of theoretical numbers.
   * https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/introduction.html#introduction-to-robot-characterization
   */
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  Field2d m_field = new Field2d();
  /*
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    switch (GameConstants.CUR_MODE){
      case CHARACTERIZATION_FLYWHEEL:
        m_autonomousCommand = new SubsystemCharacterization(m_robotContainer.mShooter, m_robotContainer.mShooter::getFlywheelRPM,
        (double rpm)->{m_robotContainer.mShooter.setDesiredFlywheelRPM(rpm);}, m_robotContainer.mShooter::stopMotors,
        5, ShooterConstants.PHYSICAL_MAX_RPM_FLYWHEEL);
        break;
      case CHARACTERIZATION_ROLLER:
        m_autonomousCommand = new SubsystemCharacterization(m_robotContainer.mShooter, m_robotContainer.mShooter::getRollerRPM,
          (double rpm)->m_robotContainer.mShooter.setDesiredRollerRPM(rpm), m_robotContainer.mShooter::stopMotors,
          5, ShooterConstants.PHYSICAL_MAX_RPM_ROLLER);
          break;
      case PID_FLYWHEEL:
        m_autonomousCommand = new SubsystemPIDTuning(m_robotContainer.mShooter.setTunablePIDFlywheel, m_robotContainer.mShooter.setFlywheelRPM(), null, null, kDefaultPeriod, kDefaultPeriod)
    }

    
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    m_robotContainer.mShooter.simulationPeriodic();
  }
}
