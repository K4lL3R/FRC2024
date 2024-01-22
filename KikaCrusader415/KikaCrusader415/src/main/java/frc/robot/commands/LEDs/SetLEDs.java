package frc.robot.commands.LEDs;

import frc.robot.subsystems.LEDs;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetLEDs extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LEDs m_subsystem;
  int mode;

  public SetLEDs(LEDs subsystem, int mode) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
    this.mode = mode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.SetLED(mode);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
