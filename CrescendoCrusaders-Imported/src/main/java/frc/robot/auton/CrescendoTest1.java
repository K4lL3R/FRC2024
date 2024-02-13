package frc.robot.auton;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.*;


public class CrescendoTest1 extends SequentialCommandGroup{
    public CrescendoTest1(String pathName) {

      Command autoPath = RobotContainer.drivetrain.getAutonPath(pathName);

      addCommands(autoPath);
    }
}
