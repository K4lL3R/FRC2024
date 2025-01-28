package frc.robot.commands.SingleSubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;

public class SetClimb extends Command {
    private Climb s_Climb;
    private Constants.Climber.Modes mode;

    public SetClimb(Climb climb,Constants.Climber.Modes mode) {
        s_Climb = climb;
        this.mode = mode;
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        s_Climb.setClimbHeight(mode);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
