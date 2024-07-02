package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class ShooterIdle extends Command {
    InOuttake.Shooter inOuttakeSubsys;
    double powerFlywheel;
    double powerHoldMotor;
  
    public ShooterIdle(InOuttake.Shooter s_shooter, double powerFlywheel, double powerHoldMotor) {
      inOuttakeSubsys = s_shooter;
      addRequirements(s_shooter);
      this.powerFlywheel = powerFlywheel;
      this.powerHoldMotor = powerHoldMotor;
    }

    @Override
    public void initialize() {
        inOuttakeSubsys.runFlyWheelsIdle(powerFlywheel);
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
