package frc.robot.commands.wrists;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class ShooterWristPos extends Command {
    Wrists.wristShooter shooter;
    Constants.Wrists.Shooter.ShooterMode pos;
  
    public ShooterWristPos(Wrists.wristShooter shooter, Constants.Wrists.Shooter.ShooterMode pos) {
      this.shooter = shooter;
      addRequirements(shooter);

      this.pos = pos;
    }

    @Override
    public void initialize() {
      shooter.wristToPositionShooter(pos);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      if (interrupted) {
        shooter.wristShooter.set(0);
      }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}
