package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.RunArmOuttake;
import frc.robot.commands.RunWristIntake;
import frc.robot.commands.wrists.ArmWristPos;
import frc.robot.commands.wrists.IntakeWristPos;
import frc.robot.commands.wrists.ShooterWristPos;


public class TrapInit extends SequentialCommandGroup {
    public TrapInit() {
        addCommands( 
                new ParallelCommandGroup(
                    new ArmWristPos(RobotContainer.s_ArmWrist, Constants.Wrists.Arm.ArmMode.Score),
                    new IntakeWristPos(RobotContainer.s_Wrist, Constants.Wrists.Intake.IntakeMode.Feed),
                    new RunWristIntake(RobotContainer.s_WristIntake, 0),
                    new RunArmOuttake(RobotContainer.s_Arm, 0),
                    new ShooterWristPos(RobotContainer.s_WristShooter, Constants.Wrists.ShooterConst.ShooterMode.Down)
                )
        );

    }
}