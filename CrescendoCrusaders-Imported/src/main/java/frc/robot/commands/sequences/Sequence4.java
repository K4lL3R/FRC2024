package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.wrists.ArmWristPos;
import frc.robot.commands.wrists.IntakeWristPos;


public class Sequence4 extends SequentialCommandGroup {
    public Sequence4(Constants.Wrists.Intake.IntakeMode intakeMode, Constants.Wrists.Arm.ArmMode mode) {
        addCommands( 
            new ParallelCommandGroup(
                new IntakeWristPos(RobotContainer.s_Wrist, intakeMode),
                new ArmWristPos(RobotContainer.s_ArmWrist, mode)
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