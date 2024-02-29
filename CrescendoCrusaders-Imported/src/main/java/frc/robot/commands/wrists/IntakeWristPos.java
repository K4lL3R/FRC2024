package frc.robot.commands.wrists;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class IntakeWristPos extends Command {
    Wrists.Wrist wristIntake;
    Constants.Wrists.Intake.IntakeMode pos;
  
    public IntakeWristPos(Wrists.Wrist wristIntake, Constants.Wrists.Intake.IntakeMode pos) {
      this.wristIntake = wristIntake;
      addRequirements(wristIntake);
      

      this.pos = pos;
    }

    @Override
    public void initialize() {
      wristIntake.wristToPositionIntake(pos);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      if (interrupted) {
        wristIntake.wristMotor.set(0);
      }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}
