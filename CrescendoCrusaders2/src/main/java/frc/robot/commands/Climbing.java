package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class Climbing extends Command {
    Climb s_Climb;
    Constants.Climb.Position pos;
  
    public Climbing(Climb climb, Constants.Climb.Position pos) {
      s_Climb = climb;
      addRequirements(climb);

      this.pos = pos;
    }

    @Override
    public void initialize() {
      s_Climb.moveClimb(pos);
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
