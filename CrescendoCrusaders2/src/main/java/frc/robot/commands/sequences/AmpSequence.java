package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Climbing;
import frc.robot.commands.RunArmOuttake;
import frc.robot.commands.RunWristIntake;
import frc.robot.commands.wrists.ArmWristPos;
import frc.robot.commands.wrists.IntakeWristPos;


public class AmpSequence extends SequentialCommandGroup {
    public AmpSequence(Constants.Climb.Position elePos, Constants.Wrists.Arm.ArmMode mode) {
        addCommands( 
                new ParallelCommandGroup(
                    new IntakeWristPos(RobotContainer.s_Wrist, Constants.Wrists.Intake.IntakeMode.Feed),
                    new AmpScore(elePos, mode),
                    new RunWristIntake(RobotContainer.s_WristIntake, 0)
                )
        );

    }
}