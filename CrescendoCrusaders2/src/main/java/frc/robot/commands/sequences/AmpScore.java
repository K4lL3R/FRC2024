package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Climbing;
import frc.robot.commands.RunArmOuttake;
import frc.robot.commands.wrists.ArmWristPos;


public class AmpScore extends SequentialCommandGroup {
    public AmpScore(Constants.Climb.Position elePos, Constants.Wrists.Arm.ArmMode mode) {
        addCommands( 
            new ParallelCommandGroup(
                new Climbing(RobotContainer.s_Climb, elePos),
                new ArmWristPos(RobotContainer.s_ArmWrist, mode),
                new RunArmOuttake(RobotContainer.s_Arm, 0)
            )
        );

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