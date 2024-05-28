package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class RunWristIntake extends Command {
    InOuttake.WristIntake inOuttakeSubsys;
    double power;
  
    public RunWristIntake(InOuttake.WristIntake s_Wrist, double power) {
      inOuttakeSubsys = s_Wrist;
      addRequirements(s_Wrist);
      this.power = power;
    }

    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        inOuttakeSubsys.runWristIntake(power);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // inOuttakeSubsys.runWristIntake(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}
