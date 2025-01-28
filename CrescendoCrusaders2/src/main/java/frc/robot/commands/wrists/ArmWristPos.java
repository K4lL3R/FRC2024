package frc.robot.commands.wrists;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class ArmWristPos extends Command {
    Wrists.armWrist wrist;
    Constants.Wrists.Arm.ArmMode pos;
  
    public ArmWristPos(Wrists.armWrist wrist, Constants.Wrists.Arm.ArmMode pos) {
      this.wrist = wrist;
      addRequirements(wrist);

      this.pos = pos;
    }

    @Override
    public void initialize() {
      wrist.wristToPositionArm(pos);

      // wrist.setSetpoint(pos);
      // wrist.wristMotorArm.set(0.2);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      if (interrupted) {
        wrist.wristMotorArm.set(0);
      }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}
