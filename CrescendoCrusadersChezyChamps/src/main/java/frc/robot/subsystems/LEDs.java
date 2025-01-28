package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    private CANdle mLeds;
    private final int LedCount = 76;

    public LEDs() {
        mLeds = new CANdle(61);
        mLeds.configLEDType(LEDStripType.GRB);

    }

        public void setAnimation(int animation) {
        switch (animation) {
            case 1:
                mLeds.animate(new StrobeAnimation(0, 255, 0, 0, 0.5, LedCount));
                break;
        
            default:
                mLeds.animate(new SingleFadeAnimation(127, 0, 255, 0, 0.9, LedCount));
                break;
                //orange = r255, g50, b0, w0
                // mLeds.animate(new FireAnimation(1, 0.8, 240, 0.8, 0.5));
                // break;
            case 3: 
                mLeds.animate(new LarsonAnimation(255, 255, 255, 0, 1, LedCount, BounceMode.Center, 7));
                break;
            case 2:
                mLeds.animate(new StrobeAnimation(0, 0, 255));
                break;
            case 4:
                mLeds.animate(new LarsonAnimation(255, 0, 0, 0, 0.7, LedCount, BounceMode.Center, 4));
                break;
            case 5:
                mLeds.animate(new LarsonAnimation(0, 255, 0, 0, 1, LedCount, BounceMode.Center, 4));
                break;
        }
    }
}
