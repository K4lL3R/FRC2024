package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import frc.robot.commands.RunWristIntake;
import frc.robot.commands.RunArmOuttake;


public class FeedSequence extends SequentialCommandGroup {
    public FeedSequence(double power) {
        if (InOuttake.ArmOuttake.beamBroken) {
            addCommands(
                new ParallelCommandGroup(
                    new RunArmOuttake(RobotContainer.s_Arm, 0),
                    new RunWristIntake(RobotContainer.s_WristIntake, 0)
                )
            );

        } else {
            addCommands(
                new ParallelCommandGroup(
                    new RunArmOuttake(RobotContainer.s_Arm, -power * 0.6),
                    new RunWristIntake(RobotContainer.s_WristIntake, power)
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