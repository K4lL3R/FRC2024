package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.RunWristIntake;
import frc.robot.commands.wrists.IntakeWristPos;
import frc.robot.subsystems.InOuttake;


public class Sequence2 extends SequentialCommandGroup {
    public Sequence2(Constants.Wrists.Intake.IntakeMode mode, double power) {
        if (InOuttake.WristIntake.beamBroken && power > 0) {
            addCommands(
                new ParallelCommandGroup(
                    new IntakeWristPos(RobotContainer.s_Wrist, Constants.Wrists.Intake.IntakeMode.Feed),
                    new RunWristIntake(RobotContainer.s_WristIntake, 0)
                    // new LEDset(RobotContainer.s_LEDs, Constants.LEDs.Colors.Green)
                )
            );
        } else {
            addCommands(
                new ParallelCommandGroup(
                    new IntakeWristPos(RobotContainer.s_Wrist, mode),
                    new RunWristIntake(RobotContainer.s_WristIntake, power)
                    // new LEDset(RobotContainer.s_LEDs, Constants.LEDs.Colors.Orange)
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