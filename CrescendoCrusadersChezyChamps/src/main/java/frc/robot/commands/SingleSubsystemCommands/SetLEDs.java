package frc.robot.commands.SingleSubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;

public class SetLEDs extends Command {
    private LEDs leds;
    private int i;

    public SetLEDs(LEDs leds, int i) {
        this.leds = leds;
        addRequirements(leds);
        this.i = i;
    }

    @Override
    public void initialize() {
        leds.setAnimation(i);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
