package frc.robot.commands.SingleSubsystemCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class Tare extends Command {
    private CommandSwerveDrivetrain swerve;

    public Tare(CommandSwerveDrivetrain swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            swerve.tare_Swerve(0);
            System.out.println("blue");
        } else {
            swerve.tare_Swerve(180);
            System.out.print("red");
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    
}
