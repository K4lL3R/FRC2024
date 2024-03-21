package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.RunShooter;
import frc.robot.commands.wrists.IntakeWristPos;

public class WristToShooter extends SequentialCommandGroup{
    public WristToShooter(double power, double holdWheel, Constants.Wrists.Intake.IntakeMode mode) {
        addCommands(
            new ParallelCommandGroup(
                new RunShooter(RobotContainer.s_ShooterOuttake, power, holdWheel),
                new IntakeWristPos(RobotContainer.s_Wrist, mode)
            )

        );
    }
    
}
