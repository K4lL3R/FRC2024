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
import frc.robot.RobotContainer;
import frc.robot.commands.ChangeAnimation;
import frc.robot.commands.RunShooter;
import frc.robot.commands.ShooterIdle;

public class ShooterWithLedsNoSwerve extends SequentialCommandGroup{
    public ShooterWithLedsNoSwerve(boolean i, double power) {
    SwerveRequest.FieldCentricFacingAngle FIELD_CENTRIC_FACING_ANGLE = new SwerveRequest.FieldCentricFacingAngle()
        .withTargetDirection(Rotation2d.fromDegrees(-34)).withRotationalDeadband(RobotContainer.MaxAngularRate * 0.01);

    SwerveRequest.FieldCentricFacingAngle FIELD_CENTRIC_FACING_ANGLE_RED = new SwerveRequest.FieldCentricFacingAngle()
        .withTargetDirection(Rotation2d.fromDegrees(34)).withRotationalDeadband(RobotContainer.MaxAngularRate * 0.01);

    FIELD_CENTRIC_FACING_ANGLE.HeadingController = new PhoenixPIDController(3, 0, 0);
    FIELD_CENTRIC_FACING_ANGLE_RED.HeadingController = new PhoenixPIDController(3, 0, 0);
        if (i) {
            addCommands(
                new ParallelCommandGroup(
                    new SequentialCommandGroup(
                        new ParallelRaceGroup(
                            new RunShooter(RobotContainer.s_ShooterOuttake, power * 0.5, power * 0.7),
                            new ChangeAnimation(RobotContainer.s_LEDs, 2),
                            new WaitCommand(0.75)
                        ),
                        new ChangeAnimation(RobotContainer.s_LEDs, 3)
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
        } else {
            addCommands(
                new ParallelCommandGroup(
                    new ChangeAnimation(RobotContainer.s_LEDs, 0),
                    new ShooterIdle(RobotContainer.s_ShooterOuttake, 0.35, 0.35)
                )
            );
        }

    }
}
