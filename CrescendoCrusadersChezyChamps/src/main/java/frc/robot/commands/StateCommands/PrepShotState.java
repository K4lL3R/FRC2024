package frc.robot.commands.StateCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.TunerConstants.DriveModes;
import frc.robot.commands.SingleSubsystemCommands.SetWristAngleManual;

public class PrepShotState extends SequentialCommandGroup{
            //turn drivertrain to angle
            //rev up shooter and run indexer after specified time
            //change shooter wrist to appropriate angle
    public PrepShotState(Constants.AngleChanger.Angles angle) {

        addRequirements(
            RobotContainer.s_ShooterWrist,
            RobotContainer.s_Swerve
        );
        addCommands(
            new ParallelCommandGroup(
                new SetWristAngleManual(RobotContainer.s_ShooterWrist, angle),
                RobotContainer.s_Swerve.setSwerveState(DriveModes.RobotCentric, RobotContainer.controller)
            )
        );
    }
}
