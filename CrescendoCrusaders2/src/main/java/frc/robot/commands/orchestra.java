package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class orchestra extends Command {
    CommandSwerveDrivetrain swerve;
  
    public orchestra(CommandSwerveDrivetrain swerve) {
      this.swerve = swerve;
      addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.orchestra.play();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}
