// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.auton.CrescendoTest1;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final CommandJoystick controller = new CommandJoystick(0);
  private final CommandJoystick buttons = new CommandJoystick(1);

  private static final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // private static final Drive s_Drive = new Drive();
  private static final Wrist s_Wrist = new Wrist();
  // private static final Shooter s_Shooter = new Shooter();
  private static final Swerve s_Swerve = new Swerve();

  // public static final AutoBuilder autoBuilder = new AutoBuilder();
  public static final SendableChooser <Command> m_chooser = new SendableChooser<>();
  public static final CrescendoTest1 shootToIntake1 = new CrescendoTest1(s_Swerve);
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // s_Drive.setDefaultCommand(new TeleopDrive(s_Drive, controller));
    // s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, controller, false));
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
  private void configureBindings() {
    m_chooser.addOption("Auto", shootToIntake1);
    SmartDashboard.putData("AutoChooser", m_chooser);
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // controller.button(13).onTrue(new spinShooter(s_Shooter, 1)).onFalse(new spinShooter(s_Shooter, 0));
    // controller.button(14).onTrue(new spinShooter(s_Shooter, -1)).onFalse(new spinShooter(s_Shooter, 0));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // controller.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    return m_chooser.getSelected();
  }
}
