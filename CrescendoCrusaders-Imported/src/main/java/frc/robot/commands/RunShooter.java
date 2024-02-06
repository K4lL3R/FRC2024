package frc.robot.commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class RunInOuttake extends Command {
    InOuttake inOuttakeSubsys;
    CANSparkMax inOuttakeMotor;
    double power;
  
    public RunInOuttake(InOuttake inOuttake, CANSparkMax inOuttakeMotor, double power) {
      inOuttakeSubsys = inOuttake;
      addRequirements(inOuttake);

      this.inOuttakeMotor = inOuttakeMotor;
      this.power = power;
    }

    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        inOuttakeSubsys.RunIntakeOuttakeMotors(inOuttakeMotor, power);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      if (InOuttake.sensor.get()) {
          inOuttakeMotor.stopMotor();
      }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}
