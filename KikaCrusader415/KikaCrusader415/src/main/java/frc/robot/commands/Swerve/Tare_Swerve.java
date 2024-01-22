package frc.robot.commands.Swerve;

import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Tare_Swerve extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Swerve m_subsystem;
  double tare_deg;

  public Tare_Swerve(Swerve subsystem, double deg) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
    this.tare_deg = deg;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.tareGyro(tare_deg);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
