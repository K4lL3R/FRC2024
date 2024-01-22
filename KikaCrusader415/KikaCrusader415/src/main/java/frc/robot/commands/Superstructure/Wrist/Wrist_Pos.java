package frc.robot.commands.Superstructure.Wrist;

import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Wrist_Pos extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Wrist m_subsystem;
  double target_deg;

  public Wrist_Pos(Wrist subsystem, double deg) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
    this.target_deg = deg;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.driveWristToDeg(target_deg);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted) {
      m_subsystem.mWrist.set(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_subsystem.wristDeg - target_deg) < 1.0;
  }
}
