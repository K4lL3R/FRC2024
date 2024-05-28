package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.commands.ChangeAnimation;
import frc.robot.commands.RunShooter;
import frc.robot.commands.RunShooterIndex;
import frc.robot.commands.RunWristIntake;
import frc.robot.subsystems.*;
import edu.wpi.first.math.*;

public class ShooterWithLeds extends SequentialCommandGroup{
    public ShooterWithLeds(boolean i, double power) {
        if (i) {
            addCommands(
                new ParallelCommandGroup(
                    new SequentialCommandGroup(
                        new ParallelRaceGroup(
                            new RunShooter(RobotContainer.s_ShooterOuttake, power, power * 0.7),
                            new ChangeAnimation(RobotContainer.s_LEDs, 2),
                            new WaitCommand(1)
                        ),
                        new ParallelRaceGroup(
                            new ChangeAnimation(RobotContainer.s_LEDs, 3),
                            new WaitCommand(0.01)
                        ),
                        new RunWristIntake(RobotContainer.s_WristIntake, 0.5)
                    ),
                    RobotContainer.drivetrain.applyRequest(() -> RobotContainer.driveRobotCentric.withVelocityX(-MathUtil.applyDeadband(RobotContainer.controller.getY(), 0.15) * RobotContainer.MaxSpeed) // Drive forward with
                    // negative Y (forward)
                        .withVelocityY(-MathUtil.applyDeadband(RobotContainer.controller.getX(), 0.15) * RobotContainer.MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(CommandSwerveDrivetrain.limelight_aim_proportional())))
                );
            // if (Math.abs(LimelightHelpers.getTX("limelight")) < 6) {
            //     addCommands(
            //         new RunWristIntake(RobotContainer.s_WristIntake, 0.5)
            //     );
            //     System.out.print("Hi");
            // }

        } else {
            addCommands(
                new ParallelCommandGroup(
                    new ChangeAnimation(RobotContainer.s_LEDs, 0),
                    new RunShooter(RobotContainer.s_ShooterOuttake, 0.25, 0),
                    new RunWristIntake(RobotContainer.s_WristIntake, 0)
                )
            );
        }

    }
}
