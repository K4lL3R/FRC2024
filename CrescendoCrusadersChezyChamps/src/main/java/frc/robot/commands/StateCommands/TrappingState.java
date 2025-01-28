package frc.robot.commands.StateCommands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.AngleChanger.Angles;
import frc.robot.Constants.Climber.Modes;
import frc.robot.Constants.ElevatorConstants.ModesEle;
import frc.robot.commands.SingleSubsystemCommands.SetClimb;
import frc.robot.commands.SingleSubsystemCommands.SetElevator;
import frc.robot.commands.SingleSubsystemCommands.SetWristAngleManual;

public class TrappingState extends SequentialCommandGroup {

    public TrappingState() {
        addRequirements(
            RobotContainer.s_Climb,
            RobotContainer.s_Indexer,
            RobotContainer.s_Elevator,
            RobotContainer.s_ShooterWrist
        );
        addCommands(
            // new ParallelCommandGroup(
            //     new SetElevator(RobotContainer.s_Elevator, ModesEle.Trap),
            //     new SetWristAngleManual(RobotContainer.s_ShooterWrist, Angles.Trap),
            //     new SetClimb(RobotContainer.s_Climb, Modes.Down)
            // )
            new SequentialCommandGroup(
                 new ParallelDeadlineGroup(
                     new WaitCommand(1),
                     new SetElevator(RobotContainer.s_Elevator, ModesEle.Trap),
                     new SetClimb(RobotContainer.s_Climb, Modes.Down)
                 ),
                 new SetWristAngleManual(RobotContainer.s_ShooterWrist, Angles.Trap)
            )

        );
        
    }
}
