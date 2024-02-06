package frc.robot.auton;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.List;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;

// import com.pathplanner.lib.*;
import java.util.HashMap;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import com.pathplanner.lib.auto.*;
// import frc.robot.commands.Superstructure.Move_To_Position;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import frc.robot.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CrescendoTest1 extends SequentialCommandGroup{
  public CrescendoTest1(Swerve swerveSubsys) {
    // List<PathPlannerTrajectory> pathGroup = new PathPlannerTrajectory();
    HashMap<String, Command> eventMap = new HashMap<>();
    AutoBuilder.configureHolonomic(
      swerveSubsys::getPose, // Pose2d supplier
      swerveSubsys::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
      () -> Constants.Swerve.swerveKinematics.toChassisSpeeds(swerveSubsys.getModuleStates()),
      swerveSubsys::setModuleStatesByChassis,
      new HolonomicPathFollowerConfig(
        new PIDConstants(5, 0, 0), 
        new PIDConstants(3,0,0), 
        4.5, 12.5, 
        new ReplanningConfig()),
      () -> true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      swerveSubsys // The drive subsystem. Used to properly set the requirements of path following commands
  );

  Command fullAuto = AutoBuilder.buildAuto("Path1");

  addCommands(fullAuto);
  }
}
