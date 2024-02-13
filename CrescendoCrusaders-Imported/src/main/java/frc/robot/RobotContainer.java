// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.auton.CrescendoTest1;
import frc.robot.commands.*;
import frc.robot.commands.sequences.FeedSequence;
import frc.robot.commands.sequences.Sequence1;
import frc.robot.commands.sequences.Sequence2;
import frc.robot.commands.sequences.Sequence3;
import frc.robot.commands.sequences.Sequence4;
import frc.robot.commands.wrists.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.InOuttake.*;
import frc.robot.subsystems.Wrists.armWrist;
import frc.robot.subsystems.Wrists.Wrist;
import frc.robot.subsystems.Wrists.wristShooter;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

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
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI;
  // The robot's subsystems and commands are defined here...
  private final CommandJoystick controller = new CommandJoystick(0);
  private final CommandJoystick buttons = new CommandJoystick(1);

  public static final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private static final CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(60);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public static final Shooter s_ShooterOuttake = new Shooter();
  public static final Climb s_Climb = new Climb();
  public static final ArmOuttake s_Arm = new ArmOuttake();
  public static final WristIntake s_WristIntake = new WristIntake();
  public static final armWrist s_ArmWrist = new armWrist();
  public static final wristShooter s_WristShooter = new wristShooter();
  public static final Wrist s_Wrist = new Wrist();
  public static final LEDs s_LEDs = new LEDs();

  // public static final AutoBuilder autoBuilder = new AutoBuilder();
  public static final SendableChooser <Command> m_chooser = new SendableChooser<>();
  public static final CrescendoTest1 autoTest = new CrescendoTest1("Path4");

  public static final Sequence1 sequence = new Sequence1(Constants.Climb.Position.Up, Constants.Wrists.ShooterConst.ShooterMode.Down);
  public static final Sequence1 sequence1pt2 = new Sequence1(Constants.Climb.Position.Down, Constants.Wrists.ShooterConst.ShooterMode.ClimbLock);
  public static final Sequence2 sequence2 = new Sequence2(Constants.Wrists.Intake.IntakeMode.Down, 0.5);
  public static final Sequence2 stopSequence2 = new Sequence2(Constants.Wrists.Intake.IntakeMode.Feed, 0);
  public static final Sequence3 sequence3Score = new Sequence3(Constants.Climb.Position.Up, Constants.Wrists.Arm.ArmMode.Score);
  public static final Sequence3 sequence3AfterAmp = new Sequence3(Constants.Climb.Position.Down, Constants.Wrists.Arm.ArmMode.Feed);
  public static final Sequence4 sequence4Feed = new Sequence4(Constants.Wrists.Intake.IntakeMode.Feed, Constants.Wrists.Arm.ArmMode.Feed);
  public static final FeedSequence feeding = new FeedSequence(-0.5);
  public static final FeedSequence stopFeed = new FeedSequence(0);

  public static final Tare_Swerve tareSwerve = new Tare_Swerve(drivetrain);
  
  public static final ArmWristPos armScore = new ArmWristPos(s_ArmWrist, Constants.Wrists.Arm.ArmMode.Score);
  public static final ArmWristPos armStow = new ArmWristPos(s_ArmWrist, Constants.Wrists.Arm.ArmMode.Stow);
  public static final ArmWristPos armFeed = new ArmWristPos(s_ArmWrist, Constants.Wrists.Arm.ArmMode.Feed);

  public static final IntakeWristPos wristFeed = new IntakeWristPos(s_Wrist, Constants.Wrists.Intake.IntakeMode.Feed);
  public static final IntakeWristPos wristStow = new IntakeWristPos(s_Wrist, Constants.Wrists.Intake.IntakeMode.Stow);
  public static final IntakeWristPos wristDown = new IntakeWristPos(s_Wrist, Constants.Wrists.Intake.IntakeMode.Down);

  public static final ShooterWristPos shootWristClimbLock = new ShooterWristPos(s_WristShooter, Constants.Wrists.ShooterConst.ShooterMode.ClimbLock);
  public static final ShooterWristPos shootWristShooting = new ShooterWristPos(s_WristShooter, Constants.Wrists.ShooterConst.ShooterMode.Shooting);
  public static final ShooterWristPos shootWristDown = new ShooterWristPos(s_WristShooter, Constants.Wrists.ShooterConst.ShooterMode.Down);
  public static final ShooterWristPos shootFromFar = new ShooterWristPos(s_WristShooter, Constants.Wrists.ShooterConst.ShooterMode.FarShots);

  public static final Climbing elevatorUp = new Climbing(s_Climb, Constants.Climb.Position.Up);
  public static final Climbing elevatorMid = new Climbing(s_Climb, Constants.Climb.Position.Mid);
  public static final Climbing elevatorDown = new Climbing(s_Climb, Constants.Climb.Position.Down);

  public static final RunWristIntake intakeDisc = new RunWristIntake(s_WristIntake, 0.2);
  public static final RunWristIntake outtakeDisc = new RunWristIntake(s_WristIntake, -0.2);
  public static final RunWristIntake stopWristIntake = new RunWristIntake(s_WristIntake, 0);

  public static final RunArmOuttake intakeArm = new RunArmOuttake(s_Arm, 1);
  public static final RunArmOuttake outtakeArm = new RunArmOuttake(s_Arm, -1);
  public static final RunArmOuttake stopArm = new RunArmOuttake(s_Arm, 0);

  public static final RunShooter spinShooter = new RunShooter(s_ShooterOuttake, 1.0, -0.7);
  public static final RunShooter stopShooter = new RunShooter(s_ShooterOuttake, 0.0, 0.0);

  // public static final RunInOuttake 

  // public static final IntakeWristPos wristDown = new IntakeWristPos(s_WristIntake, Constants.Wrists.Intake.IntakeMode.Down);
  // public static final IntakeWristPos wristStow = new IntakeWristPos(s_WristIntake, Constants.Wrists.Intake.IntakeMode.Stow);
  // public static final IntakeWristPos wristFeed = new IntakeWristPos(s_WristIntake, Constants.Wrists.Intake.IntakeMode.Feed);

  public static final LEDset LEDsGreen = new LEDset(s_LEDs, Constants.LEDs.Colors.Green);
  public static final LEDset LEDsOrange = new LEDset(s_LEDs, Constants.LEDs.Colors.Orange);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    drivetrain.tare_Swerve();
    configureBindings();
    drivetrain.getModule(0).getDriveMotor().getConfigurator().apply(currentConfigs);
    drivetrain.getModule(0).getSteerMotor().getConfigurator().apply(currentConfigs);
    drivetrain.getModule(1).getDriveMotor().getConfigurator().apply(currentConfigs);
    drivetrain.getModule(1).getSteerMotor().getConfigurator().apply(currentConfigs); 
    drivetrain.getModule(2).getDriveMotor().getConfigurator().apply(currentConfigs);
    drivetrain.getModule(2).getSteerMotor().getConfigurator().apply(currentConfigs); 
    drivetrain.getModule(3).getDriveMotor().getConfigurator().apply(currentConfigs);
    drivetrain.getModule(3).getSteerMotor().getConfigurator().apply(currentConfigs);    


    // s_Drive.setDefaultCommand(new TeleopDrive(s_Drive, controller));
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-controller.getY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-controller.getX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-controller.getZ() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));
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
    m_chooser.setDefaultOption("autoTest", autoTest);
    Trigger armBeamBreak = new Trigger(() -> s_Arm.getIsBeamBrakeBroken());
    Trigger wristBeamBreak = new Trigger(() -> s_WristIntake.getIsBeamBrakeBroken());
    // m_chooser.addOption("Auto", shootToIntake1);
    SmartDashboard.putData("AutoChooser", m_chooser);
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // controller.button(13).onTrue(new spinShooter(s_Shooter, 1)).onFalse(new spinShooter(s_Shooter, 0));
    // controller.button(14).onTrue(new spinShooter(s_Shooter, -1)).onFalse(new spinShooter(s_Shooter, 0));
    buttons.button(1).onTrue(shootWristDown);
    buttons.button(2).onTrue(shootFromFar);
    buttons.button(3).onTrue(shootWristShooting);
    buttons.button(4).onTrue(shootWristClimbLock);
    buttons.button(5).onTrue(elevatorDown);
    buttons.button(6).onTrue(elevatorUp);
    buttons.button(7).onTrue(sequence3Score);
    buttons.button(8).onTrue(stopShooter);
    buttons.button(9).onTrue(armScore);
    buttons.button(10).onTrue(sequence3AfterAmp);
    buttons.button(11).onTrue(spinShooter);
    buttons.button(12).onTrue(armFeed);
    // buttons.button(10).onTrue(LEDsGreen);
    // buttons.button(11).onTrue(LEDsOrange);

    controller.button(7).onTrue(outtakeDisc).onFalse(stopWristIntake);
    // controller.button(8).onTrue(sequence2).onFalse(sequence2Stop);
    controller.button(8).and(wristBeamBreak.negate()).whileTrue(sequence2).onFalse(stopSequence2);
    controller.button(5).onTrue(intakeArm).onFalse(stopArm);
    controller.button(6).onTrue(outtakeArm).onFalse(stopArm);
    // controller.button(3).onTrue(spinShooter).onFalse(stopShooter);
    controller.button(1).onTrue(tareSwerve);
    controller.button(2).onTrue(wristStow).onFalse(wristFeed);
    // controller.button(4).onTrue(feeding).onFalse(stopFeed);
    controller.button(3).and(armBeamBreak.negate()).whileTrue(feeding);
    controller.button(9).onTrue(LEDsGreen);

    // controller.button(3).onTrue(spinShooter);
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

  public static void configAutoCommands() {
      NamedCommands.registerCommand("IntakeDown", RobotContainer.wristDown);
      NamedCommands.registerCommand("Intake Feed to Shooter", RobotContainer.wristFeed);
      NamedCommands.registerCommand("Spin Shooter", RobotContainer.spinShooter);
      NamedCommands.registerCommand("Stop Shooter", RobotContainer.stopShooter);
      NamedCommands.registerCommand("RunIntake", RobotContainer.intakeDisc);
      NamedCommands.registerCommand("Stop Intake", RobotContainer.stopWristIntake);
      NamedCommands.registerCommand("Feed Disc to Shooter", RobotContainer.outtakeDisc);
      NamedCommands.registerCommand("ShootFromClose", RobotContainer.shootWristShooting);
      NamedCommands.registerCommand("Shoot From Far", RobotContainer.shootFromFar);
  }
}