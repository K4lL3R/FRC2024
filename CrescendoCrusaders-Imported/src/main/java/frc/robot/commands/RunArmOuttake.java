package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class RunArmOuttake extends Command {
    InOuttake.ArmOuttake inOuttakeSubsys;
    double power;
  
    public RunArmOuttake(InOuttake.ArmOuttake s_Arm, double power) {
      inOuttakeSubsys = s_Arm;
      addRequirements(s_Arm);
      this.power = power;
    }

    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        inOuttakeSubsys.runArmOuttake(power);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        inOuttakeSubsys.runArmOuttake(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}
