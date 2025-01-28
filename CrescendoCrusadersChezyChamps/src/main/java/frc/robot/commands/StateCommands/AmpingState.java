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
import frc.robot.Constants.ElevatorConstants.ModesEle;
import frc.robot.commands.SingleSubsystemCommands.RunIndexer;
import frc.robot.commands.SingleSubsystemCommands.SetElevator;
import frc.robot.commands.SingleSubsystemCommands.SetWristAngleManual;

public class AmpingState extends SequentialCommandGroup{
    public static double desiredSwerveAngleAmp;
    public AmpingState() {
    desiredSwerveAngleAmp = (DriverStation.getAlliance().get() == Alliance.Blue) ? -90 : 90;
    SwerveRequest.FieldCentricFacingAngle FIELD_CENTRIC_FACING_ANGLE = new SwerveRequest.FieldCentricFacingAngle()
        .withTargetDirection(Rotation2d.fromDegrees(desiredSwerveAngleAmp)).withRotationalDeadband(RobotContainer.MaxAngularRate * 0.01);

    FIELD_CENTRIC_FACING_ANGLE.HeadingController = new PhoenixPIDController(4, 0, 0.07);
    Command passingAngleAmp = RobotContainer.s_Swerve.applyRequest(() ->
                    FIELD_CENTRIC_FACING_ANGLE
                      .withVelocityX(-MathUtil.applyDeadband(RobotContainer.controller.getY(), 0.1) * RobotContainer.MaxSpeed)
                      .withVelocityY(-MathUtil.applyDeadband(RobotContainer.controller.getX(), 0.1) * RobotContainer.MaxSpeed));

        addRequirements(
            // RobotContainer.s_Elevator,
            RobotContainer.s_Indexer,
            RobotContainer.s_ShooterWrist
        );
        addCommands(
            new ParallelCommandGroup(
                // new ParallelRaceGroup(
                new SetElevator(RobotContainer.s_Elevator, ModesEle.Amp),
                //     new WaitCommand(0.2)
                // ),
                
                new SetWristAngleManual(RobotContainer.s_ShooterWrist, Angles.Amp),
                new RunIndexer(RobotContainer.s_Indexer, -0.5),
                passingAngleAmp
            )
        );
    }
}
