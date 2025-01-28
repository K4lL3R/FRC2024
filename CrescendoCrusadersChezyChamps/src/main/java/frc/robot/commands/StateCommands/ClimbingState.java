package frc.robot.commands.StateCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.AngleChanger.Angles;
import frc.robot.Constants.Climber.Modes;
import frc.robot.commands.SingleSubsystemCommands.SetClimb;
import frc.robot.commands.SingleSubsystemCommands.SetWristAngleManual;

public class ClimbingState extends SequentialCommandGroup{
    public ClimbingState() {
        addRequirements(
            RobotContainer.s_Climb,
            RobotContainer.s_ShooterWrist
        );
        addCommands(
            new ParallelCommandGroup(
                new SetClimb(RobotContainer.s_Climb, Modes.Climbing),
                new SetWristAngleManual(RobotContainer.s_ShooterWrist, Angles.Stow)
            )
        );

    }
}
