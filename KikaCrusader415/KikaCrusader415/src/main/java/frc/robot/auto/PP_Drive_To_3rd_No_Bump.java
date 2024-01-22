package frc.robot.auto;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.List;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Superstructure.Move_To_Position;


public class PP_Drive_To_3rd_No_Bump extends SequentialCommandGroup {
    public PP_Drive_To_3rd_No_Bump(Swerve driveSubsystem) {
        // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
        // for every path in the group
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Drive_To_3rd_No_Bump", new PathConstraints(4.3, 4.3));

        // This is just an example event map. It would be better to have a constant, global event map
        // in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Grab_Cone", new Move_To_Position(2, 1));
        eventMap.put("Grab_Cube", new Move_To_Position(2, 2));
        eventMap.put("Drop_Elevator", new Move_To_Position(0, 1));
        eventMap.put("Wrist_up", new Move_To_Position(1, 1));
        eventMap.put("Out_Cone", new Move_To_Position(0, 1));
        eventMap.put("Wrist_down", new Move_To_Position(2, 1));
        // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            driveSubsystem::getPose, // Pose2d supplier
            driveSubsystem::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
            new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(3.0, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            driveSubsystem::setModuleStates, // Module states consumer used to output to the drive subsystem
            eventMap,
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            driveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
        );

        Command fullAuto = autoBuilder.fullAuto(pathGroup);

        addCommands(fullAuto);

    }
}