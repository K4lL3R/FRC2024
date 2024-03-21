package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class RunShooterIndex extends Command {
    InOuttake.Shooter inOuttakeSubsys;
    double powerHoldMotor;
  
    public RunShooterIndex(InOuttake.Shooter s_shooter, double powerHoldMotor) {
      inOuttakeSubsys = s_shooter;
      addRequirements(s_shooter);
      this.powerHoldMotor = powerHoldMotor;
    }

    @Override
    public void initialize() {
        inOuttakeSubsys.runHoldingMotor(powerHoldMotor);
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
