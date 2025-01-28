package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private TalonFX shooterL;
    private TalonFX shooterR;

    private TalonFXConfigurator configL;
    private TalonFXConfigurator configR;

    private CurrentLimitsConfigs currentLimitsConfigs;

    private DigitalInput shooterBeamBreak;
    private boolean beamBroken;
    MotionMagicExpoVoltage m_request;
//add beam break sensor and motors
    public Shooter() {
        shooterL = new TalonFX(Constants.shooterLeftID);
        shooterR = new TalonFX(Constants.shooterRightID);

        shooterL.setInverted(true);
        shooterR.setInverted(false);
        
        configL = shooterL.getConfigurator();
        configR = shooterR.getConfigurator();

        currentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(60)
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(60);

        configL.apply(currentLimitsConfigs);
        configR.apply(currentLimitsConfigs);
        
        shooterBeamBreak = new DigitalInput(7);

                Slot0Configs configs = new Slot0Configs().withKS(0.25).withKV(0.12).withKA(0.01).withKP(0.01).withKD(0);
        // desiredPos = 0;
        
        TalonFXConfiguration wristConfig = new TalonFXConfiguration();
       MotionMagicConfigs wristMM = wristConfig.MotionMagic
            .withMotionMagicCruiseVelocity(3)
            .withMotionMagicExpo_kV(0.12)
            .withMotionMagicExpo_kA(0.1);

        shooterR.getConfigurator().apply(wristMM);
        // shooterR.getConfigurator().apply(configs);
        m_request = new MotionMagicExpoVoltage(0);
    }
    //beam break port 7

    @Override
    public void periodic() {
        beamBroken = !shooterBeamBreak.get();
        SmartDashboard.putBoolean("shooterBB", shooterBeamBreak.get());
        SmartDashboard.putNumber("shotterLspeed", getSpeedL());
        SmartDashboard.putNumber("shooterRspeed", getSpeedR());
        SmartDashboard.putNumber("shooterWheelPis", shooterL.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("shooterRpos", shooterR.getPosition().getValueAsDouble());
    }

    public void setSpeed(double speed) {
        shooterL.set(-speed * 0.425);
        shooterR.set(-speed * 0.425);
    }

    public boolean isBeamBroken() {
        return beamBroken;
    }

    public double getSpeedL() {
        return shooterL.get();
    }

    public double getSpeedR() {
        return shooterR.get();
    }

    public void setShooterWheelPos(double pos) {
        shooterR.setControl(m_request.withPosition(pos));
    }
    //-0.5, -1, -0.2
}
