package frc.robot.commands.SingleSubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class RunShooter extends Command {
    private Shooter shooter;
    private double speed;

    public RunShooter(Shooter shooter, double speed) {
        this.shooter = shooter;
        this.speed = speed;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(shooter.getSpeedL()) > Math.abs(speed * 0.95) * 0.95 && Math.abs(shooter.getSpeedR()) > Math.abs(speed) * 0.95) {
            return true;
        } else if (speed == 0) {
            return true;
        }
        return false;
    }

    
}
