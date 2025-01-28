// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LEDs;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Constants.AngleChanger.ANGLESINTERPOLATE_DOUBLE_TREE_MAP.put(38.72, -4.29);
    // Constants.AngleChanger.ANGLESINTERPOLATE_DOUBLE_TREE_MAP.put(46.0, -5.0);
    // Constants.AngleChanger.ANGLESINTERPOLATE_DOUBLE_TREE_MAP.put(57.9, -5.2);
    // Constants.AngleChanger.ANGLESINTERPOLATE_DOUBLE_TREE_MAP.put(69.95, -5.87);
    // Constants.AngleChanger.ANGLESINTERPOLATE_DOUBLE_TREE_MAP.put(81.13, -6.43);
    // Constants.AngleChanger.ANGLESINTERPOLATE_DOUBLE_TREE_MAP.put(91.56, -6.79);
    // Constants.AngleChanger.ANGLESINTERPOLATE_DOUBLE_TREE_MAP.put(103.39, -6.92);
    // Constants.AngleChanger.ANGLESINTERPOLATE_DOUBLE_TREE_MAP.put(114.64, -7.26);
    // Constants.AngleChanger.ANGLESINTERPOLATE_DOUBLE_TREE_MAP.put(121.65, -7.28);
    // Constants.AngleChanger.ANGLESINTERPOLATE_DOUBLE_TREE_MAP.put(131.66, -7.58);
    // Constants.AngleChanger.ANGLESINTERPOLATE_DOUBLE_TREE_MAP.put(150.96, -7.67);
    // Constants.AngleChanger.ANGLESINTERPOLATE_DOUBLE_TREE_MAP.put(165.28, -7.97);
    // Constants.AngleChanger.ANGLESINTERPOLATE_DOUBLE_TREE_MAP.put(183.6, -8.21);
    // Constants.AngleChanger.ANGLESINTERPOLATE_DOUBLE_TREE_MAP.put(193.28, -8.21);
    // Constants.AngleChanger.ANGLESINTERPOLATE_DOUBLE_TREE_MAP.put(203.0, -8.26);
    Constants.AngleChanger.ANGLESINTERPOLATE_DOUBLE_TREE_MAP.put(38.72, -4.8);
    Constants.AngleChanger.ANGLESINTERPOLATE_DOUBLE_TREE_MAP.put(46.0, -5.5);
    Constants.AngleChanger.ANGLESINTERPOLATE_DOUBLE_TREE_MAP.put(57.9, -5.7);
    Constants.AngleChanger.ANGLESINTERPOLATE_DOUBLE_TREE_MAP.put(69.95, -6.37);
    Constants.AngleChanger.ANGLESINTERPOLATE_DOUBLE_TREE_MAP.put(81.13, -6.93);
    Constants.AngleChanger.ANGLESINTERPOLATE_DOUBLE_TREE_MAP.put(91.56, -7.29);
    Constants.AngleChanger.ANGLESINTERPOLATE_DOUBLE_TREE_MAP.put(103.39, -7.42);
    Constants.AngleChanger.ANGLESINTERPOLATE_DOUBLE_TREE_MAP.put(114.64, -7.76);
    Constants.AngleChanger.ANGLESINTERPOLATE_DOUBLE_TREE_MAP.put(121.65, -7.78);
    Constants.AngleChanger.ANGLESINTERPOLATE_DOUBLE_TREE_MAP.put(131.66, -8.08);
    Constants.AngleChanger.ANGLESINTERPOLATE_DOUBLE_TREE_MAP.put(150.96, -8.17);
    Constants.AngleChanger.ANGLESINTERPOLATE_DOUBLE_TREE_MAP.put(165.28, -8.47);
    Constants.AngleChanger.ANGLESINTERPOLATE_DOUBLE_TREE_MAP.put(183.6, -8.71);
    Constants.AngleChanger.ANGLESINTERPOLATE_DOUBLE_TREE_MAP.put(193.28, -8.71);
    Constants.AngleChanger.ANGLESINTERPOLATE_DOUBLE_TREE_MAP.put(203.0, -8.76);
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
  public void disabledInit() {
    new LEDs().setAnimation(0);
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // int[] idArray = {1, 2,3,4,5,6,7,8,9,10,11,12,13,14,15};
    // LimelightHelpers.SetFiducialIDFiltersOverride("limelight-upfront", idArray);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

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
    // int[] idArray = {4,7};
    // LimelightHelpers.SetFiducialIDFiltersOverride("limelight-upfront", idArray);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.teleopShooterBeamBreak();
    // if (DriverStation.getAlliance().get() == Alliance.Red) {
    //   RobotContainer.s_Swerve.tare_Swerve(180);
    // }
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
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
