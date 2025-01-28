package frc.robot.commands.StateCommands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.AngleChanger.Angles;
import frc.robot.Constants.ElevatorConstants.ModesEle;
import frc.robot.commands.SingleSubsystemCommands.RunIndexer;
import frc.robot.commands.SingleSubsystemCommands.RunIntake;
import frc.robot.commands.SingleSubsystemCommands.RunShooter;
import frc.robot.commands.SingleSubsystemCommands.SetElevator;
import frc.robot.commands.SingleSubsystemCommands.SetWristAngleManual;

public class DefaultState extends SequentialCommandGroup{
    public DefaultState() {
        addRequirements(
            RobotContainer.s_Climb, RobotContainer.s_Indexer, RobotContainer.s_Intake, RobotContainer.s_Shooter,
             RobotContainer.s_ShooterWrist
        );

        addCommands(
            new ParallelDeadlineGroup(
                new WaitCommand(0.7),
                // new SetClimb(RobotContainer.s_Climb, Modes.Down),
                new SetElevator(RobotContainer.s_Elevator, ModesEle.Down),
                // new SetWristAngleManual(RobotContainer.s_ShooterWrist, Constants.AngleChanger.Angles.Feed),
                new RunIndexer(RobotContainer.s_Indexer, 0),
                new RunIntake(RobotContainer.s_Intake, 0),
                new RunShooter(RobotContainer.s_Shooter, 0),
                // new ParallelRaceGroup(
                new SetWristAngleManual(RobotContainer.s_ShooterWrist, Angles.Feed)
                    // new WaitCommand(0.5)
                // )
                // new SetLEDs(RobotContainer.s_leds, 0)
                // RobotContainer.s_leds.getDefaultCommand()
            )
        );
    }

    
}
