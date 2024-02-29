package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleFaults;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LEDs.AnimationTypes;

public class LEDs extends SubsystemBase {
    private final int ledLength = 70;
    private final int ledPort = 7;
    private final AddressableLED object_led = new AddressableLED(ledPort);
    private final AddressableLEDBuffer object_ledBuffer = new AddressableLEDBuffer(ledLength);
    // public static AddressableLED mLEDs;
    private final int LedCount = 32;
    private final CANdle mLEDs;
    


    // public LEDs() {
        //  mLEDs.setData(m_ledBuffer);

        // mLEDs.configLEDType(LEDStripType.RGB);

                // mLEDs.setLEDs(0, 254, 0);

                // RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.8, 50);
                // mLEDs.animate(rainbowAnim);
                // StrobeAnimation strobeAnim = new StrobeAnimation(254, 0, 0, 0, 0.2, 54, 3);
                // mLEDs.animate(strobeAnim);
                // FireAnimation fireAnim = new FireAnimation(0.6, 0.7, 54, 0.3, 0.3, false, 10);
                // mLEDs.animate(fireAnim);
        
                // changeAnimation(AnimationTypes.SetAll);
                // CANdleConfiguration configAll = new CANdleConfiguration();
                // configAll.statusLedOffWhenActive = true;
                // configAll.disableWhenLOS = false;
                // configAll.brightnessScalar = 0.5;
                // configAll.vBatOutputMode = VBatOutputMode.Modulated;
                // mLEDs.configAllSettings(configAll, 100);
        
                // ErrorCode error = mLEDs.getLastError();
                // CANdleFaults faults = new CANdleFaults();
                // ErrorCode faultsError = mLEDs.getFaults(faults);
    // }

    // public void setGreen() {
    //     for (var i = 0; i < ledLength; i++) {
    //         object_ledBuffer.setRGB(i, 0, 255, 0);
    //     }
    //     object_led.setData(object_ledBuffer);
    // }

    // public void incrementAnimation() {
    //     switch(m_currentAnimation) {
    //         case ColorFlow: changeAnimation(AnimationTypes.Fire); break;
    //         case Fire: changeAnimation(AnimationTypes.Larson); break;
    //         case Larson: changeAnimation(AnimationTypes.Rainbow); break;
    //         case Rainbow: changeAnimation(AnimationTypes.RgbFade); break;
    //         case RgbFade: changeAnimation(AnimationTypes.SingleFade); break;
    //         case SingleFade: changeAnimation(AnimationTypes.Strobe); break;
    //         case Strobe: changeAnimation(AnimationTypes.Twinkle); break;
    //         case Twinkle: changeAnimation(AnimationTypes.TwinkleOff); break;
    //         case TwinkleOff: changeAnimation(AnimationTypes.ColorFlow); break;
    //         case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
    //         }
    //     }
    // public void decrementAnimation() {
    //     switch(m_currentAnimation) {
    //         case ColorFlow: changeAnimation(AnimationTypes.TwinkleOff); break;
    //         case Fire: changeAnimation(AnimationTypes.ColorFlow); break;
    //         case Larson: changeAnimation(AnimationTypes.Fire); break;
    //         case Rainbow: changeAnimation(AnimationTypes.Larson); break;
    //         case RgbFade: changeAnimation(AnimationTypes.Rainbow); break;
    //         case SingleFade: changeAnimation(AnimationTypes.RgbFade); break;
    //         case Strobe: changeAnimation(AnimationTypes.SingleFade); break;
    //         case Twinkle: changeAnimation(AnimationTypes.Strobe); break;
    //         case TwinkleOff: changeAnimation(AnimationTypes.Twinkle); break;
    //         case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
    //     }
    // }
    // public void setColors() {
    //     changeAnimation(AnimationTypes.SetAll);
    // }

    // /* Wrappers so we can access the CANdle from the subsystem */
    
    // public void changeAnimation(AnimationTypes toChange) {
    //     m_currentAnimation = toChange;
        
    //     switch(toChange)
    //     {
    //         case Fire:
                // m_toAnimate = new FireAnimation(0.5, 0.7, LedCount, 0.7, 0.5);
    //             break;
    //         case Larson:
    //             m_toAnimate = new LarsonAnimation(0, 255, 46, 0, 1, LedCount, BounceMode.Front, 3);
    //             break;
    //         case Rainbow:
    //             m_toAnimate = new RainbowAnimation(1, 0.1, LedCount);
    //             break;
    //         case RgbFade:
    //             m_toAnimate = new RgbFadeAnimation(0.7, 0.4, LedCount);
    //             break;
    //         case SingleFade:
    //             m_toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LedCount);
    //             break;
    //         case Strobe:
    //             m_toAnimate = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, LedCount);
    //             break;
    //         case Twinkle:
    //             m_toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, LedCount, TwinklePercent.Percent6);
    //             break;
    //         case TwinkleOff:
    //             m_toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.8, LedCount, TwinkleOffPercent.Percent100);
    //             break;
    //         case SetAll:
    //             m_toAnimate = null;
    //             break;
    //     }
    // }

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
        
            case 0:
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