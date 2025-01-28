package frc.robot.commands.StateCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.AngleChanger.Angles;
import frc.robot.commands.SingleSubsystemCommands.RunIndexer;
import frc.robot.commands.SingleSubsystemCommands.RunIntake;
import frc.robot.commands.SingleSubsystemCommands.SetWristAngleManual;

public class IntakingState extends SequentialCommandGroup {
    public IntakingState() {
        addRequirements(
            RobotContainer.s_Intake,
            RobotContainer.s_Indexer,
            RobotContainer.s_ShooterWrist
        );
        addCommands(
            new SequentialCommandGroup(
                new ParallelRaceGroup(
                    new SetWristAngleManual(RobotContainer.s_ShooterWrist, Angles.Feed),
                    new WaitCommand(0.35)
                    // new SetLEDs(RobotContainer.s_leds, 1)
                ),
                
                new ParallelCommandGroup(
                    new RunIndexer(RobotContainer.s_Indexer, -0.3),
                    new RunIntake(RobotContainer.s_Intake, 1)
                )
            )            
        );

    }
}
