package frc.robot.auto;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Superstructure.Intake.*;
import frc.robot.commands.Swerve.Tare_Swerve;
import frc.robot.commands.Superstructure.Move_To_Position;

public class Three_Bump extends SequentialCommandGroup {
    public Three_Bump(Swerve driveSubsystem) {
        addCommands(
            new Tare_Swerve(RobotContainer.s_Swerve, 0),
            new Move_To_Position(0, 3),
            new WaitCommand(0.5),
            new ParallelRaceGroup(
                new Run_Intake(RobotContainer.s_Intake, -0.75),
                new WaitCommand(0.5)
            ),
            new ParallelRaceGroup(
                new Run_Intake(RobotContainer.s_Intake, 0),
                new WaitCommand(0.05)    
            ),
            //new Move_To_Position(0, 1),
            new PP_Two_Object_Bump(RobotContainer.s_Swerve),
            new ParallelRaceGroup(
                new Run_Intake(RobotContainer.s_Intake, 0.8),
                new WaitCommand(1)
            ),
            new ParallelRaceGroup(
                new Run_Intake(RobotContainer.s_Intake, 0),
                new WaitCommand(0.05)    
            ),
            //new Move_To_Position(0, 1),
            new PP_Drive_To_3rd_Bump(RobotContainer.s_Swerve),
            new ParallelRaceGroup(
                new Run_Intake(RobotContainer.s_Intake, 0.8),
                new WaitCommand(1)
            ),
            new Tare_Swerve(RobotContainer.s_Swerve, 180) //Modified 0 - 180 9/18 11:55 am
        );
    }
}