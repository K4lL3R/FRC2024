package frc.robot.commands.Superstructure;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Superstructure.Elevator.*;
import frc.robot.commands.Superstructure.Wrist.*;
import frc.robot.commands.Superstructure.Intake.*;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class Move_To_Position extends SequentialCommandGroup {
    public Move_To_Position(int mode, int height) {
        if(mode == 0) {
            if(height == 3) {
                addCommands(
                    new ParallelCommandGroup(
                        new Elevator_Pos(RobotContainer.s_Elevator, Constants.Elevator.ConeSetpoints.high),
                        new SequentialCommandGroup(
                            new WaitCommand(0.25),
                            new Wrist_Pos(RobotContainer.s_Wrist, Constants.Wrist.ConeSetpoints.high)
                        )
                    )
                );
            }
            else if(height == 2) {
                addCommands(
                    new ParallelCommandGroup(
                        new Elevator_Pos(RobotContainer.s_Elevator, Constants.Elevator.ConeSetpoints.mid),
                        new Wrist_Pos(RobotContainer.s_Wrist, Constants.Wrist.ConeSetpoints.mid)
                    )
                );
            }
            else {
                addCommands(
                    new ParallelCommandGroup(
                        new Elevator_Pos(RobotContainer.s_Elevator, Constants.Elevator.GlobalSetpoints.ret),
                        new Wrist_Pos(RobotContainer.s_Wrist, Constants.Wrist.GlobalSetpoints.stow)
                    )
                );
            }
        }
        else if(mode == 1) {
            if(height == 3) {
                addCommands(
                    new ParallelCommandGroup(
                        new Elevator_Pos(RobotContainer.s_Elevator, Constants.Elevator.CubeSetpoints.high),
                        new SequentialCommandGroup(
                            new WaitCommand(0.25),
                            new Wrist_Pos(RobotContainer.s_Wrist, Constants.Wrist.CubeSetpoints.high)
                        )
                    )
                );
            }
            else if(height == 2) {
                addCommands(
                    new ParallelCommandGroup(
                        new Elevator_Pos(RobotContainer.s_Elevator, Constants.Elevator.CubeSetpoints.mid),
                        new Wrist_Pos(RobotContainer.s_Wrist, Constants.Wrist.CubeSetpoints.mid)
                    )
                );
            }
            else {
                addCommands(
                    new ParallelCommandGroup(
                        new Elevator_Pos(RobotContainer.s_Elevator, Constants.Elevator.GlobalSetpoints.ret),
                        new Wrist_Pos(RobotContainer.s_Wrist, Constants.Wrist.GlobalSetpoints.stow)
                    )
                );
            }
        }
        else if(mode == 2) {
            if(height == 0) {
                addCommands(
                    new ParallelCommandGroup(
                        new Elevator_Pos(RobotContainer.s_Elevator, Constants.Elevator.GlobalSetpoints.ret),
                        new Wrist_Pos(RobotContainer.s_Wrist, Constants.Wrist.GlobalSetpoints.stow),
                        new Run_Intake(RobotContainer.s_Intake, 0)
                    )
                );
            }
            else if(height == 1) {
                addCommands(
                    new ParallelCommandGroup(
                        new Elevator_Pos(RobotContainer.s_Elevator, Constants.Elevator.GlobalSetpoints.intake_cone),
                        new Wrist_Pos(RobotContainer.s_Wrist, Constants.Wrist.GlobalSetpoints.intake_cone),
                        new Run_Intake(RobotContainer.s_Intake, Constants.Intake.intake_cone_outtake_cube)
                    )
                );
            }
            else if(height == 2) {
                addCommands(
                    new ParallelCommandGroup(
                        new Elevator_Pos(RobotContainer.s_Elevator, Constants.Elevator.GlobalSetpoints.intake_cube),
                        new Wrist_Pos(RobotContainer.s_Wrist, Constants.Wrist.GlobalSetpoints.intake_cube),
                        new Run_Intake(RobotContainer.s_Intake, Constants.Intake.intake_cube_outtake_cone)
                    )
                );
            }
            else {
                addCommands(
                    new ParallelCommandGroup(
                        new Elevator_Pos(RobotContainer.s_Elevator, Constants.Elevator.GlobalSetpoints.intake_cone_feeder),
                        new Wrist_Pos(RobotContainer.s_Wrist, Constants.Wrist.GlobalSetpoints.intake_cone_feeder),
                        new Run_Intake(RobotContainer.s_Intake, Constants.Intake.intake_cone_outtake_cube)
                    )
                );
            }
        }
    }
}