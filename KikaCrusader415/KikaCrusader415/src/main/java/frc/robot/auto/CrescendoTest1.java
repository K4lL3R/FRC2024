// package frc.robot.auto;

// import frc.robot.Constants;
// import frc.robot.subsystems.Swerve;
// import java.util.List;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import com.pathplanner.lib.*;
// import java.util.HashMap;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.commands.Superstructure.Move_To_Position;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// public class CrescendoTest1 extends SequentialCommandGroup{
//   public CrescendoTest1(Swerve swerveSubsys) {
//     List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Path1", new PathConstraints(3, 3));
//     HashMap<String, Command> eventMap = new HashMap<>();
//     SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
//       swerveSubsys::getPose, // Pose2d supplier
//       swerveSubsys::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
//       Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
//       new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
//       new PIDConstants(3.0, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
//       swerveSubsys::setModuleStates, // Module states consumer used to output to the drive subsystem
//       eventMap,
//       true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
//       swerveSubsys // The drive subsystem. Used to properly set the requirements of path following commands
//   );

//   Command fullAuto = autoBuilder.fullAuto(pathGroup);

//   addCommands(fullAuto);
//   }
// }
