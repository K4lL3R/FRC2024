package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.ChangeAnimation;
import frc.robot.commands.RunShooter;

public class ShooterWithLeds extends SequentialCommandGroup{
    public ShooterWithLeds(boolean i) {
        if (i) {
            addCommands(
            new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        new RunShooter(RobotContainer.s_ShooterOuttake, 1, -0.7),
                        new ChangeAnimation(RobotContainer.s_LEDs, 2),
                        new WaitCommand(0.75)
                    ),
                    new ChangeAnimation(RobotContainer.s_LEDs, 3)
                )


            );
        } else {
            addCommands(
                new ParallelCommandGroup(
                    new ChangeAnimation(RobotContainer.s_LEDs, 0),
                    new RunShooter(RobotContainer.s_ShooterOuttake, 0.12, 0)
                )
            );
        }

    }
}
