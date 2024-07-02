package frc.robot.commands.sequences;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.RunWristIntake;
import frc.robot.commands.wrists.ArmWristPos;
import frc.robot.commands.wrists.IntakeWristPos;
import frc.robot.commands.Climbing;
import frc.robot.commands.RunArmOuttake;


public class AmpFeedSequenceButtonPanel extends SequentialCommandGroup {
    public AmpFeedSequenceButtonPanel(double power) {

    SwerveRequest.FieldCentricFacingAngle FIELD_CENTRIC_FACING_ANGLE = new SwerveRequest.FieldCentricFacingAngle()
        .withTargetDirection(Rotation2d.fromDegrees(90)).withRotationalDeadband(RobotContainer.MaxAngularRate * 0.01);

    SwerveRequest.FieldCentricFacingAngle FIELD_CENTRIC_FACING_ANGLE_RED = new SwerveRequest.FieldCentricFacingAngle()
        .withTargetDirection(Rotation2d.fromDegrees(-90)).withRotationalDeadband(RobotContainer.MaxAngularRate * 0.01);

    FIELD_CENTRIC_FACING_ANGLE.HeadingController = new PhoenixPIDController(3.3, 0, 0);
    FIELD_CENTRIC_FACING_ANGLE_RED.HeadingController = new PhoenixPIDController(3.3, 0, 0);
        if (power == 0) {
            addCommands(
                new ParallelCommandGroup(
                    new ArmWristPos(RobotContainer.s_ArmWrist, Constants.Wrists.Arm.ArmMode.Feed),
                    new IntakeWristPos(RobotContainer.s_Wrist, Constants.Wrists.Intake.IntakeMode.Feed),
                    new RunArmOuttake(RobotContainer.s_Arm, power),
                    new RunWristIntake(RobotContainer.s_WristIntake, power)
                    
                )
            );
        } else {
            addCommands(
                new ParallelCommandGroup(
                    new SequentialCommandGroup(
                         new ParallelRaceGroup(
                             new ArmWristPos(RobotContainer.s_ArmWrist, Constants.Wrists.Arm.ArmMode.Stow),
                             new IntakeWristPos(RobotContainer.s_Wrist, Constants.Wrists.Intake.IntakeMode.Stow),
                             new Climbing(RobotContainer.s_Climb, Constants.Climb.Position.Down),
                             new WaitCommand(0.5)
                         ),
                         new ParallelCommandGroup(
                             new RunArmOuttake(RobotContainer.s_Arm, power * 1.7),
                             new RunWristIntake(RobotContainer.s_WristIntake, power * 1.6)
                         )
                     ),

                RobotContainer.drivetrain.applyRequest(() -> DriverStation.getAlliance().get() == Alliance.Blue ? 
                    FIELD_CENTRIC_FACING_ANGLE
                      .withVelocityX(-MathUtil.applyDeadband(RobotContainer.controller.getY(), 0.1) * RobotContainer.MaxSpeed)
                      .withVelocityY(-MathUtil.applyDeadband(RobotContainer.controller.getX(), 0.1) * RobotContainer.MaxSpeed) : 
                    FIELD_CENTRIC_FACING_ANGLE_RED
                      .withVelocityX(-MathUtil.applyDeadband(RobotContainer.controller.getY(), 0.1) * RobotContainer.MaxSpeed)
                      .withVelocityY(-MathUtil.applyDeadband(RobotContainer.controller.getX(), 0.1) * RobotContainer.MaxSpeed))
                )

            );
        }
    }
}

//add a file for each sequence (wrist to pos, then start shooter speed up, etc.)

//sequence 1: shooter seq
//shooter wrist angle change; set reference to pos
//intake wrist to feed position


//climb ready: shoot down, intake up
//on chain: 
//2 leds, 1 just elevator up + shoot down, 1 elevator to amp height, 1 button feed from intake to ele-arm
//1 ele down, 1 elevator to trap height, 1 shoot up