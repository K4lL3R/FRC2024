package frc.robot.commands.StateCommands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.AngleChanger.Angles;
import frc.robot.commands.SingleSubsystemCommands.RunShooter;
import frc.robot.commands.SingleSubsystemCommands.SetWristAngleManual;

public class PassingState extends SequentialCommandGroup{
    public static double desiredSwerveAngle;
    public PassingState() {
    desiredSwerveAngle = (DriverStation.getAlliance().get() == Alliance.Blue) ? -34 : 34;
    SwerveRequest.FieldCentricFacingAngle FIELD_CENTRIC_FACING_ANGLE = new SwerveRequest.FieldCentricFacingAngle()
        .withTargetDirection(Rotation2d.fromDegrees(desiredSwerveAngle)).withRotationalDeadband(RobotContainer.MaxAngularRate * 0.01);

    FIELD_CENTRIC_FACING_ANGLE.HeadingController = new PhoenixPIDController(4, 0, 0.07);
    Command passingAngle = RobotContainer.s_Swerve.applyRequest(() ->
                    FIELD_CENTRIC_FACING_ANGLE
                      .withVelocityX(-MathUtil.applyDeadband(RobotContainer.controller.getY(), 0.1) * RobotContainer.MaxSpeed)
                      .withVelocityY(-MathUtil.applyDeadband(RobotContainer.controller.getX(), 0.1) * RobotContainer.MaxSpeed));


        
    
        // if (i) {
        addCommands(
            new ParallelCommandGroup(
                new SetWristAngleManual(RobotContainer.s_ShooterWrist, Angles.Pass),
                new RunShooter(RobotContainer.s_Shooter, 0.6),
                passingAngle
            )
        );
        
        // } else {
        //     addCommands(
        //         new ParallelCommandGroup(
        //             new ChangeAnimation(RobotContainer.s_LEDs, 0),
        //             new ShooterIdle(RobotContainer.s_ShooterOuttake, 0.35, 0.35)
        //         ) its just the intake
        //     );
        // }

    }
}
