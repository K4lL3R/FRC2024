package frc.robot.commands.SingleSubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants.ModesEle;
import frc.robot.subsystems.Elevator;

public class SetElevator extends Command{
    private Elevator elevator;
    private Constants.ElevatorConstants.ModesEle mode;
    
    public SetElevator(Elevator s_Elevator, Constants.ElevatorConstants.ModesEle mode) {
        elevator = s_Elevator;
        this.mode = mode;

        addRequirements(s_Elevator);
    }

    @Override
    public void initialize() {
        elevator.setEleHeight(mode);
    }

    @Override
    public void execute() {
        //
    }

    @Override
    public boolean isFinished() {
        if (mode == ModesEle.Down) {
            if (elevator.getEleLPos() <= Constants.ElevatorConstants.EleSetpoints.downPos) {
                return true;
            }
        }
        // if (Math.abs(elevator.getEleSpeed()) < 0.2) {
        //     return true;
        // }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (isFinished() == true) {
            elevator.stopMotors();
        }
    }
}