package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
    // public static AddressableLED mLEDs;
    private final int LedCount = 32; 
    private final CANdle mLEDs;

    public LEDs() {
        mLEDs = new CANdle(61);
        mLEDs.configLEDType(LEDStripType.GRB);
    }

    public void setLEDs(Constants.LEDs.Colors color) {
        switch (color) {
            case Green: mLEDs.setLEDs(120, 0, 255);
            break;
            case Orange: mLEDs.setLEDs(155, 30, 30);
            break;
        }
    }

    public void setAnimation(int animation) {
        switch (animation) {
            case 1:
                mLEDs.animate(new RainbowAnimation(1, 1, LedCount));
                break;
        
            default:
                mLEDs.animate(new LarsonAnimation(255, 50, 0, 0, 0.7, LedCount, BounceMode.Center, 4));
                break;
            case 3: 
                mLEDs.animate(new LarsonAnimation(255, 255, 255, 0, 1, LedCount, BounceMode.Center, 7));
                break;
            case 2:
                mLEDs.animate(new LarsonAnimation(255, 0, 0, 0, 0.7, LedCount, BounceMode.Center, 4));
                break;
            case 4:
                mLEDs.animate(new LarsonAnimation(255, 0, 255, 0, 0.7, LedCount, BounceMode.Center, 4));
                break;
            case 5:
                mLEDs.animate(new LarsonAnimation(0, 255, 0, 0, 1, LedCount, BounceMode.Center, 4));
                break;
        }
    }
}
//config and methods to run each intake/outtake motor
//outtake from arm for trap, run shooter, intake from wrist
//may need delay for shooter to spin up (here or in the sequences files)
//sensor to detect if note has been launched?