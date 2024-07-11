package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.ChangeAnimation;
import frc.robot.commands.RunShooter;
import frc.robot.subsystems.*;

public class ShooterWithLedsAuto extends SequentialCommandGroup{
    public ShooterWithLedsAuto(boolean i, double power) {
        if (i) {
            addCommands(
                new ParallelCommandGroup(
                    new SequentialCommandGroup(
                        new ParallelRaceGroup(
                            new RunShooter(RobotContainer.s_ShooterOuttake, power, power * 0.7),
                            new ChangeAnimation(RobotContainer.s_LEDs, 2),
                            new WaitCommand(1)
                        ),
                        new ChangeAnimation(RobotContainer.s_LEDs, 3)
                    ),
                    RobotContainer.drivetrain.applyRequest(() -> RobotContainer.driveRobotCentric.withRotationalRate(CommandSwerveDrivetrain.limelight_aim_proportional()))
                )
            );
        } else {
            addCommands(
                new ParallelCommandGroup(
                    new ChangeAnimation(RobotContainer.s_LEDs, 0),
                    new RunShooter(RobotContainer.s_ShooterOuttake, 0.25, 0)
                )
            );
        }

    }
}
