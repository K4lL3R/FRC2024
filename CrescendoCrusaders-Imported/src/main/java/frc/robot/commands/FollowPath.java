package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.*;


public class FollowPath extends SequentialCommandGroup{
    public FollowPath() {
      addCommands(RobotContainer.drivetrain.findPath());
    }
}
