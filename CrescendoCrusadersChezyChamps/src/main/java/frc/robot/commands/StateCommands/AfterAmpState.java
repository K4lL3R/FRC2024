package frc.robot.commands.StateCommands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.SingleSubsystemCommands.RunIndexer;

public class AfterAmpState extends SequentialCommandGroup{
    public AfterAmpState() {
        addRequirements(
            RobotContainer.s_Indexer
        );
        addCommands(
            new ParallelDeadlineGroup(
                new WaitCommand(0.45),
                new RunIndexer(RobotContainer.s_Indexer, 1)
                
            )
        );
    }
}
