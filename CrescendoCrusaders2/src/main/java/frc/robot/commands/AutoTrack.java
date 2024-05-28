package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrists.wristShooter;

public class AutoTrack extends Command{
    wristShooter subsys;

    public AutoTrack(wristShooter shoot) {
        subsys = shoot;
        addRequirements(shoot);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        subsys.interpolatedAngleAuto(wristShooter.distance);
    }

    @Override
    public boolean isFinished() {
        return false;
    }   
}
