package frc.robot.commands.SingleSubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command {
    private Intake s_Intake;
    private double speed;

    public RunIntake(Intake intake, double speed) {
        s_Intake = intake;
        this.speed = speed;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        s_Intake.setSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
