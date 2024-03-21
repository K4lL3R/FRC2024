package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;
// import frc.robot.subsystems.LEDs.AnimationTypes;

public class LEDset extends Command {
    LEDs LEDsubsys;
    Constants.LEDs.Colors color;
  
    public LEDset(LEDs mLED, Constants.LEDs.Colors color) {
      LEDsubsys = mLED;
      addRequirements(mLED);

      this.color = color;
    }

    @Override
    public void initialize() {
       LEDsubsys.setLEDs(color);
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
        return false;
    }
}
