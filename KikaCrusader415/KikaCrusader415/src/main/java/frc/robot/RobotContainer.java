package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.*;
import frc.robot.commands.Swerve.*;
import frc.robot.commands.Superstructure.*;
import frc.robot.commands.Superstructure.Intake.Run_Intake;
import frc.robot.commands.LEDs.*;
import frc.robot.auto.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public final CommandJoystick driverController = new CommandJoystick(0);
  public final CommandJoystick buttonPanel = new CommandJoystick(1);
  
  public static final Swerve s_Swerve = new Swerve();
  public static final Elevator s_Elevator = new Elevator();
  public static final Wrist s_Wrist = new Wrist();
  public static final Intake s_Intake = new Intake();
  public static final LEDs s_LEDs = new LEDs();

  public final Tare_Swerve tare_swerve_0 = new Tare_Swerve(s_Swerve, 0);
  public final Tare_Swerve tare_swerve_90 = new Tare_Swerve(s_Swerve, 90);
  public final Tare_Swerve tare_swerve_neg_90 = new Tare_Swerve(s_Swerve, -90);
  public final Tare_Swerve tare_swerve_180 = new Tare_Swerve(s_Swerve, 180);

  public final Move_To_Position cone_high = new Move_To_Position(0, 3);
  public final Move_To_Position cone_mid = new Move_To_Position(0, 2);
  public final Move_To_Position cone_low = new Move_To_Position(0, 1);

  public final Move_To_Position cube_high = new Move_To_Position(1, 3);
  public final Move_To_Position cube_mid = new Move_To_Position(1, 2);
  public final Move_To_Position cube_low = new Move_To_Position(1, 1);

  public final Move_To_Position stow_superstructure = new Move_To_Position(2, 0);
  public final Move_To_Position intake_ground_cone = new Move_To_Position(2, 1);
  public final Move_To_Position intake_ground_cube = new Move_To_Position(2, 2);
  public final Move_To_Position intake_feeder_cone = new Move_To_Position(2, 3);

  public final Run_Intake stop_intake = new Run_Intake(s_Intake, 0);
  public final Run_Intake intake_cube_outtake_cone = new Run_Intake(s_Intake, Constants.Intake.intake_cube_outtake_cone);
  public final Run_Intake intake_cone_outtake_cube = new Run_Intake(s_Intake, Constants.Intake.intake_cone_outtake_cube);
  
  public final SetLEDs set_led_purple = new SetLEDs(s_LEDs, 1);
  public final SetLEDs set_led_yellow = new SetLEDs(s_LEDs, 2);
  public final SetLEDs set_led_red = new SetLEDs(s_LEDs, 3);

  //Auto
  public static final SendableChooser <Command> m_chooser = new SendableChooser<>();
  public static final Two_Half_No_Bump two_half_no_bump = new Two_Half_No_Bump(s_Swerve);
  public static final Three_Bump Three_Bump = new Three_Bump(s_Swerve);
  public static final Balance_Taxi Balance_Taxi = new Balance_Taxi(s_Swerve);
  public static final Two_Balance_No_Bump two_balance_no_bump = new Two_Balance_No_Bump(s_Swerve);
  public static final Three_No_Bump Three_no_bump = new Three_No_Bump(s_Swerve);
  // public static final CrescendoTest1 shootToIntake1 = new CrescendoTest1(s_Swerve);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    s_Swerve.setDefaultCommand(
            new TeleopSwerve(s_Swerve, driverController, false)
        );

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
    m_chooser.setDefaultOption("Balance Taxi", Balance_Taxi);
    m_chooser.addOption("Three Bump", Three_Bump);
    //m_chooser.addOption("Two Balance No Bump", two_balance_no_bump);
    m_chooser.addOption("Three Clear", Three_no_bump);
    //m_chooser.addOption("Balance Taxi", Balance_Taxi);
    SmartDashboard.putData("AUTO", m_chooser);

    driverController.button(5).onTrue(intake_cube_outtake_cone).onFalse(stop_intake);
    driverController.button(6).onTrue(intake_cone_outtake_cube).onFalse(stop_intake);
    driverController.button(7).onTrue(intake_ground_cube).onFalse(stow_superstructure);
    driverController.button(8).onTrue(intake_ground_cone).onFalse(stow_superstructure);
    driverController.button(3).onTrue(intake_feeder_cone).onFalse(stow_superstructure);

    driverController.button(14).onTrue(tare_swerve_180);
    

    buttonPanel.button(4).onTrue(cone_high);
    buttonPanel.button(7).onTrue(cone_mid);
    buttonPanel.button(10).onTrue(cone_low);
    buttonPanel.button(5).onTrue(cube_high);
    buttonPanel.button(8).onTrue(cube_mid);
    buttonPanel.button(11).onTrue(cube_low);
    buttonPanel.button(6).onTrue(set_led_red);
    buttonPanel.button(9).onTrue(set_led_yellow);
    buttonPanel.button(12).onTrue(set_led_purple);
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
