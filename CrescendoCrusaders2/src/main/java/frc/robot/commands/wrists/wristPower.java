package frc.robot.commands.wrists;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrists.wristShooter;

public class wristPower extends Command{
    wristShooter subsys;

    public wristPower(wristShooter shoot) {
        subsys = shoot;
        addRequirements(shoot);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        subsys.interpolatedAngle(wristShooter.distance);
    }

    @Override
    public boolean isFinished() {
        return false;
    }   
}
