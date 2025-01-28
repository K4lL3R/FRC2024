package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.States;

public class SetRobotState extends Command {
    private States.RobotState state;

    public SetRobotState(States.RobotState state) {
        this.state = state;
    }

    @Override
    public void initialize() {
        States.run(state);
    }
}
