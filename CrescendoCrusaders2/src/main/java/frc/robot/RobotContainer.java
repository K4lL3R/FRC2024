// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.auton.CrescendoTest1;
import frc.robot.commands.*;
import frc.robot.commands.sequences.*;
import frc.robot.commands.wrists.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.InOuttake.*;
import frc.robot.subsystems.Wrists.*;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;

import com.pathplanner.lib.auto.*;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //speed constants
  public static double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  public static double MaxAngularRate = 3 * Math.PI;///was 1.5
  // controller and button panel ports
  public static final CommandJoystick controller = new CommandJoystick(0);
  public static final CommandJoystick buttons = new CommandJoystick(1);





  //swerve + current
  public static final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private static final CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs()
  .withSupplyCurrentLimitEnable(true)
  .withSupplyCurrentLimit(60)
  .withStatorCurrentLimitEnable(true)
  .withStatorCurrentLimit(60);

  //fieldcentric
  public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.03) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  //robotcentric for tracking
  public static final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.03) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);



  //subsystems
  public static final Shooter s_ShooterOuttake = new Shooter();
  public static final Climb s_Climb = new Climb();
  public static final ArmOuttake s_Arm = new ArmOuttake();
  public static final WristIntake s_WristIntake = new WristIntake();
  public static final armWrist s_ArmWrist = new armWrist();
  public static final wristShooter s_WristShooter = new wristShooter();
  public static final Wrist s_Wrist = new Wrist();
  public static final LEDs s_LEDs = new LEDs();

  public static SendableChooser <Command> m_chooser;

  //sequences
  public static final ClimbSequence sequence = new ClimbSequence(Constants.Climb.Position.Up, Constants.Wrists.ShooterConst.ShooterMode.Down);
  public static final ClimbSequence sequence1pt2 = new ClimbSequence(Constants.Climb.Position.Down, Constants.Wrists.ShooterConst.ShooterMode.ClimbLock);
  public static final IntakingSequence sequence2 = new IntakingSequence(Constants.Wrists.Intake.IntakeMode.Down, -0.5);
  public static final IntakingSequence stopSequence2 = new IntakingSequence(Constants.Wrists.Intake.IntakeMode.Feed, 0);
  public static final AmpScore sequence3Score = new AmpScore(Constants.Climb.Position.Amp, Constants.Wrists.Arm.ArmMode.Score);
  public static final AmpScore sequence3AfterAmp = new AmpScore(Constants.Climb.Position.Down, Constants.Wrists.Arm.ArmMode.Feed);
  public static final WristArmFeedPos sequence4Feed = new WristArmFeedPos(Constants.Wrists.Intake.IntakeMode.Feed, Constants.Wrists.Arm.ArmMode.Feed);
  public static final FeedSequence feeding = new FeedSequence(-0.5);
  public static final FeedSequence stopFeed = new FeedSequence(0);
  public static final WristToShooter feedToShooter = new WristToShooter(1, -0.7, Constants.Wrists.Intake.IntakeMode.Feed);
  public static final WristToShooter stopShooterFeed = new WristToShooter(0, 0, Constants.Wrists.Intake.IntakeMode.Stow);
  public static final ShooterWithLeds spinShot = new ShooterWithLeds(true, 1);
  public static final ShooterWithLeds stopShot = new ShooterWithLeds(false, 0);

  //tare
  public static final Tare_Swerve tareSwerve = new Tare_Swerve(drivetrain, 0);
  public static final Tare_Swerve tareSwerve270 = new Tare_Swerve(drivetrain, 270);
  public static final Tare_Swerve tareSwerve90 = new Tare_Swerve(drivetrain, 90);
  public static final Tare_Swerve tareSwerve180 = new Tare_Swerve(drivetrain, 180);
  
  //arm position
  public static final ArmWristPos armScore = new ArmWristPos(s_ArmWrist, Constants.Wrists.Arm.ArmMode.Score);
  public static final ArmWristPos armStow = new ArmWristPos(s_ArmWrist, Constants.Wrists.Arm.ArmMode.Stow);
  public static final ArmWristPos armFeed = new ArmWristPos(s_ArmWrist, Constants.Wrists.Arm.ArmMode.Feed);

  //intake wrist pos
  public static final IntakeWristPos wristFeed = new IntakeWristPos(s_Wrist, Constants.Wrists.Intake.IntakeMode.Feed);
  public static final IntakeWristPos wristStow = new IntakeWristPos(s_Wrist, Constants.Wrists.Intake.IntakeMode.Stow);
  public static final IntakeWristPos wristDown = new IntakeWristPos(s_Wrist, Constants.Wrists.Intake.IntakeMode.Down);

  //shooter wrist pos
  public static final ShooterWristPos shootWristClimbLock = new ShooterWristPos(s_WristShooter, Constants.Wrists.ShooterConst.ShooterMode.ClimbLock);
  public static final ShooterWristPos shootWristShooting = new ShooterWristPos(s_WristShooter, Constants.Wrists.ShooterConst.ShooterMode.Shooting);
  public static final ShooterWristPos shootWristDown = new ShooterWristPos(s_WristShooter, Constants.Wrists.ShooterConst.ShooterMode.Down);
  public static final ShooterWristPos shootFromFar = new ShooterWristPos(s_WristShooter, Constants.Wrists.ShooterConst.ShooterMode.FarShots);

  //elevator
  public static final Climbing elevatorUp = new Climbing(s_Climb, Constants.Climb.Position.Up);
  public static final Climbing elevatorMid = new Climbing(s_Climb, Constants.Climb.Position.Mid);
  public static final Climbing elevatorDown = new Climbing(s_Climb, Constants.Climb.Position.Down);

  //spin wrist intake
  public static final RunWristIntake intakeDisc = new RunWristIntake(s_WristIntake, -0.5);
  public static final RunWristIntake outtakeDisc = new RunWristIntake(s_WristIntake, 0.5);
  public static final RunWristIntake stopWristIntake = new RunWristIntake(s_WristIntake, 0);

  //spin arm intake/outtake
  public static final RunArmOuttake intakeArm = new RunArmOuttake(s_Arm, 1);
  public static final RunArmOuttake outtakeArm = new RunArmOuttake(s_Arm, 0.1);
  public static final RunArmOuttake stopArm = new RunArmOuttake(s_Arm, 0);

  //run shooter
  public static final RunShooter spinShooter = new RunShooter(s_ShooterOuttake, 1.0, -0.7);
  public static final RunShooter stopShooter = new RunShooter(s_ShooterOuttake, 0.0, 0.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    NamedCommands.registerCommand("w", new IntakeWristPos(s_Wrist, Constants.Wrists.Intake.IntakeMode.Down));
    NamedCommands.registerCommand("WristUp", new IntakeWristPos(s_Wrist, Constants.Wrists.Intake.IntakeMode.Feed));
    NamedCommands.registerCommand("SpinShooter", new RunShooter(s_ShooterOuttake, 1, 0.7));
    NamedCommands.registerCommand("StopShooter", new RunShooter(s_ShooterOuttake, 0, 0));
    NamedCommands.registerCommand("1", new RunWristIntake(s_WristIntake, 0.5));
    NamedCommands.registerCommand("2", new RunWristIntake(s_WristIntake, -0.5));
    NamedCommands.registerCommand("3", new RunWristIntake(s_WristIntake, 0));
    NamedCommands.registerCommand("ShootHigh", new ShooterWristPos(s_WristShooter, Constants.Wrists.ShooterConst.ShooterMode.Shooting));
    NamedCommands.registerCommand("ShootLock", new ShooterWristPos(s_WristShooter, Constants.Wrists.ShooterConst.ShooterMode.ClimbLock));
    NamedCommands.registerCommand("ShootFar", new ShooterWristPos(s_WristShooter, Constants.Wrists.ShooterConst.ShooterMode.AutoFar));
    NamedCommands.registerCommand("Tare180", new Tare_Swerve(drivetrain, 180));
    NamedCommands.registerCommand("Track", new AutoTrack(s_WristShooter));
    NamedCommands.registerCommand("Indexer", new RunShooterIndex(s_ShooterOuttake, 0.7));

    drivetrain.configPathPlanner();
    m_chooser = new SendableChooser<>();

    m_chooser.setDefaultOption("5 Note First", new CrescendoTest1("5 Note First"));
    m_chooser.addOption("4Note", new CrescendoTest1("4 Note Auto"));
    m_chooser.addOption("973", new CrescendoTest1("973"));
    m_chooser.addOption("973_mew", new CrescendoTest1("973_mew"));
    m_chooser.addOption("ClearTrack", new CrescendoTest1("ClearTrack"));
    m_chooser.addOption("6 Note", new CrescendoTest1("6 Note"));
    m_chooser.addOption("New Auto", new CrescendoTest1("New Auto"));
    //m_chooser.addOption("5D 1-2R", new CrescendoTest1("5D 1-2R"));
    //m_chooser.addOption("Copy of 5D 1-2R", new CrescendoTest1("Copy of 5D 1-2R"));

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-MathUtil.applyDeadband(controller.getY(), 0.1) * RobotContainer.MaxSpeed) // Drive forward with
        // negative Y (forward)
          .withVelocityY(-MathUtil.applyDeadband(controller.getX(), 0.1) * RobotContainer.MaxSpeed) // Drive left with negative X (left)
          .withRotationalRate(-MathUtil.applyDeadband(controller.getZ(), 0.1) * RobotContainer.MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));
    configureBindings();
    SmartDashboard.putData("Auto", m_chooser);
    drivetrain.getModule(0).getDriveMotor().getConfigurator().apply(currentConfigs);
    drivetrain.getModule(0).getSteerMotor().getConfigurator().apply(currentConfigs);
    drivetrain.getModule(1).getDriveMotor().getConfigurator().apply(currentConfigs);
    drivetrain.getModule(1).getSteerMotor().getConfigurator().apply(currentConfigs); 
    drivetrain.getModule(2).getDriveMotor().getConfigurator().apply(currentConfigs);
    drivetrain.getModule(2).getSteerMotor().getConfigurator().apply(currentConfigs); 
    drivetrain.getModule(3).getDriveMotor().getConfigurator().apply(currentConfigs);
    drivetrain.getModule(3).getSteerMotor().getConfigurator().apply(currentConfigs);
    drivetrain.configNeutralMode(NeutralModeValue.Brake);  

    drivetrain.getPigeon2().reset();
    s_LEDs.setDefaultCommand(new ChangeAnimation(s_LEDs, 0));

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
    Trigger armBeamBreak = new Trigger(() -> s_Arm.getIsBeamBrakeBroken());
    Trigger wristBeamBreak = new Trigger(() -> s_WristIntake.getIsBeamBrakeBroken());
    // Trigger aprilTagDetected = new Trigger(() -> LimelightHelpers.getFiducialID("limelight") == 4 || LimelightHelpers.getFiducialID("limelight") == 7);
    // wristBeamBreak.onTrue(new ChangeAnimation(s_LEDs, 1));
    buttons.button(1).onTrue(shootWristDown);
    buttons.button(11).onTrue(new wristPower(s_WristShooter));
    buttons.button(3).onTrue(shootWristShooting.alongWith(new ShooterWithLedsNoSwerve(true, 1)));
    buttons.button(4).onTrue(shootWristClimbLock);
    buttons.button(5).onTrue(elevatorDown);
    buttons.button(6).onTrue(elevatorUp);
    buttons.button(7).onTrue(sequence3Score);
    controller.button(7).onTrue(spinShot).onFalse(stopShot);
    // buttons.button(11).and(limelightTX).onTrue(spinShot.alongWith(outtakeDisc)).onFalse(stopShooter.alongWith(stopWristIntake));
    buttons.button(9).and(armBeamBreak.negate()).onTrue(new ChangeAnimation(s_LEDs, 5).alongWith(new TrapPassThrough(0.5))).onFalse(new TrapInit());
    buttons.button(10).onTrue(Commands.runOnce(() -> drivetrain.orchestra.play(), drivetrain)).onFalse(drivetrain.getDefaultCommand());
    // buttons.button(8).and(armTrigger.negate()).onTrue(new ChangeAnimation(s_LEDs, 5).alongWith(new AmpFeedSequenceButtonPanel(0.5))).onFalse(new AmpSequence(Constants.Climb.Position.Amp, Constants.Wrists.Arm.ArmMode.Score));
    buttons.button(8).onTrue(new AmpSeq(0.5));
    buttons.button(2).onTrue(shootFromFar);
    buttons.button(12).onTrue(new ArmWristPos(s_ArmWrist, Constants.Wrists.Arm.ArmMode.Demo));

    controller.button(8).whileTrue(sequence2.alongWith(new ShooterWithLedsNoSwerve(false, 0))).onFalse(stopSequence2);
    controller.button(5).onTrue(intakeArm).onFalse(sequence3AfterAmp);
    controller.pov(0).onTrue(outtakeArm).onFalse(stopArm);
    controller.button(1).onTrue(tareSwerve);
    controller.button(2).and(wristBeamBreak.negate()).onTrue(new ChangeAnimation(s_LEDs, 5).alongWith(new AmpFeedSequenceButtonPanel(-0.3))).onFalse(new AmpFeedSequenceButtonPanel(0));
    controller.pov(180).onTrue(intakeArm).onFalse(stopArm);
    controller.button(6).onTrue(outtakeDisc).onFalse(stopWristIntake);
    controller.button(3).whileTrue(        

        drivetrain.applyRequest(() -> driveRobotCentric.withVelocityX(-MathUtil.applyDeadband(controller.getY(), 0.1) * RobotContainer.MaxSpeed)
        .withVelocityY(-MathUtil.applyDeadband(controller.getX(), 0.1) * RobotContainer.MaxSpeed) 
        .withRotationalRate(-MathUtil.applyDeadband(controller.getZ(), 0.1) * RobotContainer.MaxAngularRate)))
    .onFalse(drivetrain.getDefaultCommand());

  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}