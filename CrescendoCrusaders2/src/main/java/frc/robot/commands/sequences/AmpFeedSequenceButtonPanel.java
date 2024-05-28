package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.RunWristIntake;
import frc.robot.commands.wrists.ArmWristPos;
import frc.robot.commands.wrists.IntakeWristPos;
import frc.robot.commands.RunArmOuttake;


public class AmpFeedSequenceButtonPanel extends SequentialCommandGroup {
    public AmpFeedSequenceButtonPanel(double power) {
        if (power == 0) {
            addCommands(
                new ParallelCommandGroup(
                    new ArmWristPos(RobotContainer.s_ArmWrist, Constants.Wrists.Arm.ArmMode.Feed),
                    new IntakeWristPos(RobotContainer.s_Wrist, Constants.Wrists.Intake.IntakeMode.Feed),
                    new RunArmOuttake(RobotContainer.s_Arm, power),
                    new RunWristIntake(RobotContainer.s_WristIntake, power)
                )
            );
        } else {
            addCommands(
                new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        new ArmWristPos(RobotContainer.s_ArmWrist, Constants.Wrists.Arm.ArmMode.Stow),
                        new IntakeWristPos(RobotContainer.s_Wrist, Constants.Wrists.Intake.IntakeMode.Stow),
                        new WaitCommand(0.5)
                    ),
                    new ParallelCommandGroup(
                        new RunArmOuttake(RobotContainer.s_Arm, power * 1.7),
                        new RunWristIntake(RobotContainer.s_WristIntake, power * 1.6)
                    )
                )
            );
        }
    }
}

//add a file for each sequence (wrist to pos, then start shooter speed up, etc.)

//sequence 1: shooter seq
//shooter wrist angle change; set reference to pos
//intake wrist to feed position


//climb ready: shoot down, intake up
//on chain: 
//2 leds, 1 just elevator up + shoot down, 1 elevator to amp height, 1 button feed from intake to ele-arm
//1 ele down, 1 elevator to trap height, 1 shoot up