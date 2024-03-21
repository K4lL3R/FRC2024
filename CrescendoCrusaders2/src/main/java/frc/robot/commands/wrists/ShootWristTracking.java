package frc.robot.commands.wrists;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class ShootWristTracking extends Command {
    Wrists.wristShooter shooter;
  
    public ShootWristTracking(Wrists.wristShooter shooter) {
      this.shooter = shooter;
      addRequirements(shooter);
    }

    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      shooter.setShooterAngle(Wrists.wristShooter.distance);
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
