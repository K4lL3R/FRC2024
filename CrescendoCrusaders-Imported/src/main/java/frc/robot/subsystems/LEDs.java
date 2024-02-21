package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
    public static CANdle mLEDs;

    public LEDs() {
        mLEDs = new CANdle(61);
        mLEDs.configLEDType(LEDStripType.RGB);
    }

    public void setLEDs(Constants.LEDs.Colors color) {
        switch (color) {
            case Green: mLEDs.setLEDs(5, 10, 30);
            break;
            case Orange: mLEDs.setLEDs(155, 10, 30);
            break;
        }
    }
}
//config and methods to run each intake/outtake motor
//outtake from arm for trap, run shooter, intake from wrist
//may need delay for shooter to spin up (here or in the sequences files)
//sensor to detect if note has been launched?