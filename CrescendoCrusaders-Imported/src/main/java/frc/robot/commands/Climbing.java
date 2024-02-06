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
      if(interrupted) {
        s_Climb.climbMotorL.set(0);
        s_Climb.climbMotorR.set(0);
      }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      if (pos == Constants.Climb.Position.Up) {
        return Math.abs(s_Climb.climbMotorL.getEncoder().getPosition() - Constants.Climb.GlobalSetpoints.elevatorUpPos) < 1000
                && Math.abs(s_Climb.climbMotorR.getEncoder().getPosition() - Constants.Climb.GlobalSetpoints.elevatorUpPos) < 1000;
      } else {
        return Math.abs(s_Climb.climbMotorR.getEncoder().getPosition() - Constants.Climb.GlobalSetpoints.elevatorDefaultPos) < 1000
                && Math.abs(s_Climb.climbMotorR.getEncoder().getPosition() - Constants.Climb.GlobalSetpoints.elevatorUpPos) < 1000;
      }
    }
}
