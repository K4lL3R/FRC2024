// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.AngleChanger.Angles;
import frc.robot.Constants.Climber.Modes;
import frc.robot.Constants.TunerConstants.DriveModes;
import frc.robot.commands.Auton;
import frc.robot.commands.SingleSubsystemCommands.RunIndexer;
import frc.robot.commands.SingleSubsystemCommands.RunShooter;
import frc.robot.commands.SingleSubsystemCommands.SetClimb;
import frc.robot.commands.SingleSubsystemCommands.SetLEDs;
import frc.robot.commands.SingleSubsystemCommands.SetWristAngleManual;
import frc.robot.commands.SingleSubsystemCommands.Tare;
import frc.robot.commands.StateCommands.IntakingState;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterWrist;
import frc.robot.subsystems.States;
import frc.robot.subsystems.States.RobotState;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  public static double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps * 0.7; // kSpeedAt12VoltsMps desired top speed
  public static double MaxAngularRate = 3 * Math.PI * 0.7;
  // The robot's subsystems and commands are defined here...
  //Swerve, Shooter, Intake, AngleChanger, Climb, Indexer
  public static final Climb s_Climb = new Climb();
  public static final Indexer s_Indexer = new Indexer();
  public static final Intake s_Intake = new Intake();
  public static final Shooter s_Shooter = new Shooter();
  public static final ShooterWrist s_ShooterWrist = new ShooterWrist();
  public static final Elevator s_Elevator = new Elevator();
  public static final CommandSwerveDrivetrain s_Swerve = Constants.TunerConstants.DriveTrain;
  public static final LEDs s_leds = new LEDs();
  public static final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
  .withDeadband(RobotContainer.MaxSpeed * 0.05).withRotationalDeadband(RobotContainer.MaxAngularRate * 0.03) // Add a 10% deadband
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
  .withDeadband(RobotContainer.MaxSpeed * 0.05).withRotationalDeadband(RobotContainer.MaxAngularRate * 0.01) // Add a 10% deadband
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final CommandJoystick controller = new CommandJoystick(0);
  private static final CommandJoystick buttons = new CommandJoystick(1);
  // private static final XboxController x = new XboxController(2);
  

   private static SendableChooser <Command> m_chooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    NamedCommands.registerCommand("IntakeState", new IntakingState());
    NamedCommands.registerCommand("ShooterWheelsSpin", new RunShooter(s_Shooter, 1));
    NamedCommands.registerCommand("SpinIndexer", new RunIndexer(s_Indexer, -1));
    NamedCommands.registerCommand("WristWoofer", new SetWristAngleManual(s_ShooterWrist, Angles.Woofer));
    NamedCommands.registerCommand("Wrist", new SetWristAngleManual(s_ShooterWrist, Angles.CloseTruss));
    NamedCommands.registerCommand("StopShooter", new RunShooter(s_Shooter, 0));
    NamedCommands.registerCommand("StopIndexer", new RunIndexer(s_Indexer, 0));
    NamedCommands.registerCommand("ReverseIndexer", new RunIndexer(s_Indexer, 0.5));
    NamedCommands.registerCommand("KnockDisc", new SetWristAngleManual(s_ShooterWrist, Angles.Amp));
    NamedCommands.registerCommand("ReverseShooter", new RunShooter(s_Shooter, -0.2));

    s_Swerve.configPathPlanner();
    m_chooser = new SendableChooser<>();
    m_chooser.setDefaultOption("4 note", new Auton("4"));
    m_chooser.addOption("Source", new Auton("Source"));
    m_chooser.addOption("Preload", new Auton("Preload"));
    configureBindings();
    SmartDashboard.putData("Auto", m_chooser);

    // s_Swerve.setDefaultCommand(s_Swerve.applyRequest
    // (() -> drive.withVelocityX(-MathUtil.applyDeadband(controller.getY(), 0.1) * RobotContainer.MaxSpeed) // Drive forward with
    // // negative Y (forward)
    //   .withVelocityY(-MathUtil.applyDeadband(controller.getX(), 0.1) * RobotContainer.MaxSpeed) // Drive left with negative X (left)
    //   .withRotationalRate(-MathUtil.applyDeadband(controller.getZ(), 0.1) * RobotContainer.MaxAngularRate))); // Drive counterclockwise with negative X (left)));
    // s_Swerve.setDefaultCommand(States.run(RobotState.Driving));
    s_Swerve.setDefaultCommand(s_Swerve.setSwerveState(DriveModes.FieldCentric, controller));
    // s_leds.setDefaultCommand(new SetLEDs(s_leds, 0));
    
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

    controller.pov(0).onTrue(new RunIndexer(s_Indexer, 0.19)).onFalse(new RunIndexer(s_Indexer, 0));
    controller.pov(180).onTrue(new RunIndexer(s_Indexer, -0.19)).onFalse(new RunIndexer(s_Indexer, 0));
    controller.pov(90).onTrue(Commands.runOnce(() -> s_Elevator.elePower(0.05))).onFalse(Commands.runOnce(() -> s_Elevator.elePower(0.03)));
    controller.pov(270).onTrue(Commands.runOnce(() -> s_Elevator.elePower(-0.03))).onFalse(Commands.runOnce(() -> s_Elevator.elePower(0.03)));
    controller.axisGreaterThan(2, 0.1).onTrue(States.run(RobotState.Shooting)).onFalse(States.run(RobotState.Default));
    controller.axisGreaterThan(3, 0.1).onTrue(States.run(RobotState.Intaking)).onFalse(States.run(RobotState.Default));

    controller.button(1).whileTrue(s_Swerve.applyRequest(() -> driveRobotCentric.withVelocityX(MathUtil.applyDeadband(controller.getY(), 0.1) * RobotContainer.MaxSpeed) // Drive forward with
    // negative Y (forward)
      .withVelocityY(MathUtil.applyDeadband(controller.getX(), 0.1) * RobotContainer.MaxSpeed) // Drive left with negative X (left)
      .withRotationalRate(-MathUtil.applyDeadband(controller.getRawAxis(4), 0.1) * RobotContainer.MaxAngularRate)));
    controller.button(3).onTrue(States.run(RobotState.Passing));
    controller.button(4).onTrue(new RunIndexer(s_Indexer, -1)).onFalse(States.run(RobotState.Default));
    controller.button(5).onTrue(States.run(RobotState.ManualWoofer)).onFalse(States.run(RobotState.Default));
    controller.button(6).onTrue(States.run(RobotState.Amping)).onFalse(States.run(RobotState.AfterAmp).andThen(States.run(RobotState.Default)));
    controller.button(8).onTrue(new Tare(s_Swerve));

    // buttons.button(9).onTrue(new SetElevator(s_Elevator, ModesEle.Down));

    // buttons.button(5).onTrue(new SetWristPosTracking(s_ShooterWrist));
    // buttons.button(10).onTrue(new SetWristPosTracking(s_ShooterWrist));
    buttons.button(11).onTrue(Commands.runOnce(() -> s_Intake.setSpeed(-1))).onFalse(Commands.runOnce(() -> s_Intake.setSpeed(0)));
    buttons.button(8).onTrue(States.run(RobotState.Climbing));
    buttons.button(7).onTrue(new SetClimb(s_Climb, Modes.Mid));
    buttons.button(6).onTrue(States.run(RobotState.Trapping));
    // buttons.button(4).onTrue(new SetClimb(s_Climb, Modes.Zero));
  }

  public void teleopShooterBeamBreak() {
      Trigger intakeTrigger = new Trigger(() -> s_Intake.isBeamBroken());
        intakeTrigger.onTrue(new SetLEDs(s_leds, 1));
      Trigger shooterTrigger = new Trigger(() -> s_Shooter.isBeamBroken());
        shooterTrigger.onTrue
        (new RunIndexer(s_Indexer, 0).alongWith
        (new SetLEDs(s_leds, 2))).onFalse(new SetLEDs(s_leds, 0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_chooser.getSelected();
  }
}
