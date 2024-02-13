package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Tare_Swerve extends Command{
    CommandSwerveDrivetrain swerveSubsys;

    public Tare_Swerve(CommandSwerveDrivetrain swerve) {
        swerveSubsys = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerveSubsys.tare_Swerve();
    }

    @Override
    public boolean isFinished() {
        return true;
    }   
}
