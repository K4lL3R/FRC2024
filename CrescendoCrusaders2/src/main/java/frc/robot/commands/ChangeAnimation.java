package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
// import frc.robot.subsystems.LEDs.AnimationTypes;

public class ChangeAnimation extends Command {
    LEDs LEDsubsys;
    int i;
  
    public ChangeAnimation(LEDs mLED, int i) {
      LEDsubsys = mLED;
      addRequirements(mLED);

      this.i = i;
    }

    @Override
    public void initialize() {
      LEDsubsys.setAnimation(i);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        if (i == 0) {
          return true;
        }
        return false;
    }
}
