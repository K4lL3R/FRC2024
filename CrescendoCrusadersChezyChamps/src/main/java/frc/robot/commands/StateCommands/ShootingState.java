package frc.robot.commands.StateCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.AngleChanger.Angles;
import frc.robot.commands.SingleSubsystemCommands.RunIndexer;
import frc.robot.commands.SingleSubsystemCommands.RunShooter;
import frc.robot.commands.SingleSubsystemCommands.SetLEDs;
import frc.robot.commands.SingleSubsystemCommands.SetWristAngleManual;
import frc.robot.commands.SingleSubsystemCommands.SetWristPosTracking;
import frc.robot.subsystems.Vision;

public class ShootingState extends SequentialCommandGroup{
    public ShootingState() {
        addRequirements(
            RobotContainer.s_Shooter,
            RobotContainer.s_Indexer
        );
        addCommands(
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        // new WaitCommand(0.5),
                        new ParallelCommandGroup(
                            new RunShooter(RobotContainer.s_Shooter, 1),
                            new SetWristAngleManual(RobotContainer.s_ShooterWrist, Angles.CloseTruss)             
                        ),
                        new SetLEDs(RobotContainer.s_leds, 4),
                         new WaitCommand(0.4)

                    ),
                    new WaitCommand(0.21),
                    new ParallelCommandGroup(
                        new RunIndexer(RobotContainer.s_Indexer, -1),
                        new SetLEDs(RobotContainer.s_leds, 3)
                    )

                ),
                RobotContainer.s_Swerve.applyRequest(() -> RobotContainer.driveRobotCentric.withVelocityX(-MathUtil.applyDeadband(RobotContainer.controller.getY(), 0.15) * RobotContainer.MaxSpeed) // Drive forward with
                    // negative Y (forward)
                        .withVelocityY(-MathUtil.applyDeadband(RobotContainer.controller.getX(), 0.15) * RobotContainer.MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(Vision.limelight_aim_proportional())))
            );


    }
}
