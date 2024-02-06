package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Climbing;
import frc.robot.commands.wrists.ShooterWristPos;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Wrists.wristShooter;

public class Sequence1 extends SequentialCommandGroup {


    public Sequence1() {
        new SequentialCommandGroup(
            new Climbing(new Climb(), Constants.Climb.Position.Up),
            new ShooterWristPos(new wristShooter(), Constants.Wrists.Shooter.ShooterMode.Down)
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