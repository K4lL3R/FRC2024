package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Tare_Swerve extends Command{
    CommandSwerveDrivetrain swerveSubsys;
    double deg;

    public Tare_Swerve(CommandSwerveDrivetrain swerve, double deg) {
        swerveSubsys = swerve;
        addRequirements(swerve);
        this.deg = deg;
    }

    @Override
    public void initialize() {
        swerveSubsys.tare_Swerve(deg);
    }

    @Override
    public boolean isFinished() {
        return true;
    }   
}
