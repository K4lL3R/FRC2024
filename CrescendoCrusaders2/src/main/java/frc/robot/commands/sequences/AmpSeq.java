package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Climbing;
import frc.robot.commands.RunArmOuttake;
import frc.robot.commands.RunWristIntake;
import frc.robot.commands.wrists.ArmWristPos;
import frc.robot.commands.wrists.IntakeWristPos;

public class AmpSeq extends SequentialCommandGroup{
    public AmpSeq(double power) {
        addCommands(
            new SequentialCommandGroup(
            new ParallelRaceGroup(
                new ArmWristPos(RobotContainer.s_ArmWrist, Constants.Wrists.Arm.ArmMode.Stow),
                new IntakeWristPos(RobotContainer.s_Wrist, Constants.Wrists.Intake.IntakeMode.Stow),
                new Climbing(RobotContainer.s_Climb, Constants.Climb.Position.Down),
                new WaitCommand(0.5)
            ),
            new ParallelRaceGroup(
                new RunArmOuttake(RobotContainer.s_Arm, power * 1.7),
                new RunWristIntake(RobotContainer.s_WristIntake, power * 1.6),
                new WaitCommand(0.1)
            ),
            new AmpSequence(Constants.Climb.Position.Amp, Constants.Wrists.Arm.ArmMode.Score)
        )
        );


    }
}
