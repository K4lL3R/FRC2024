package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
    public static Spark mLEDs;

    public LEDs() {
        mLEDs = new Spark(0);
    }

    public void setLEDs(Constants.LEDs.Colors color) {
        switch (color) {
            case Green: mLEDs.set(0);
            break;
            case Orange: mLEDs.set(1);
            break;
        }
    }
}
//config and methods to run each intake/outtake motor
//outtake from arm for trap, run shooter, intake from wrist
//may need delay for shooter to spin up (here or in the sequences files)
//sensor to detect if note has been launched?