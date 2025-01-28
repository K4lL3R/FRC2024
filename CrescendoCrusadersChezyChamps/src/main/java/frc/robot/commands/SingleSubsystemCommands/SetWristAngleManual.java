package frc.robot.commands.SingleSubsystemCommands;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterWrist;

public class SetWristAngleManual extends Command {
    private ShooterWrist wrist;
    private Constants.AngleChanger.Angles mode;
    // private double initerror;
    // private double error;
    
    public SetWristAngleManual(ShooterWrist s_Wrist, Constants.AngleChanger.Angles mode) {
        wrist = s_Wrist;
        this.mode = mode;

        addRequirements(s_Wrist);
        // error = 0;
    }

    @Override
    public void initialize() {
        wrist.setDesiredPos(mode);
        // initerror = (wrist.desiredPos > wrist.getShooterPos()) ? (wrist.desiredPos - wrist.getShooterPos()) : (wrist.getShooterPos() - wrist.desiredPos);
        // // wrist.spinToDesiredPos();
        // SmartDashboard.putNumber("init", initerror);
    }

    @Override
    public void execute() {
        // error = (wrist.desiredPos > wrist.getShooterPos()) ? (wrist.desiredPos - wrist.getShooterPos()) : (wrist.getShooterPos() - wrist.desiredPos);
        // SmartDashboard.putBoolean("isWristFinished", isFinished());
        // SmartDashboard.putNumber("error", error);
        
    }

    @Override
    public boolean isFinished() {
        // if (error < initerror / 4) {
        //     return true;
        // }
        // if (Math.abs(wrist.getWristSpeed()) < 0.17) {
        //     return true;
        // }
        if (wrist.inPosition()) {
            return true;
        }
        return false;
    }
}
