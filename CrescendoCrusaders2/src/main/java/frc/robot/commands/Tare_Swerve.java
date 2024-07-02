package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                swerveSubsys.tare_Swerve(180);
            } else {
                swerveSubsys.tare_Swerve(0);
            }
    }

    @Override
    public boolean isFinished() {
        return true;
    }   
}
