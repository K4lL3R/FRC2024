package frc.robot.commands.Superstructure.Elevator;

import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Elevator_Pos extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Elevator m_subsystem;
  double target_height;

  public Elevator_Pos(Elevator subsystem, double height) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
    this.target_height = height;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.elevatorPIDController.setSmartMotionMaxVelocity(Constants.Elevator.maxVel, 0);
    m_subsystem.elevatorPIDController.setSmartMotionMaxAccel(Constants.Elevator.maxAccel, 0);
    m_subsystem.driveElevatorToHeight(target_height);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted) {
      m_subsystem.mElevator.set(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_subsystem.elevatorHeight - target_height) < 2.0;
  }
}
