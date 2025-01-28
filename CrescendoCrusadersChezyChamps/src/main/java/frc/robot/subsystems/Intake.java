package frc.robot.subsystems;


import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private TalonFX intake;

    // private TalonFXConfigurator configIntake;

    // private CurrentLimitsConfigs currentLimitsConfigs;

    private DigitalInput intakeBeamBreak;
    private boolean beamBroken;
    private TalonFXConfiguration intakeConfig;

    // private static final CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs()
    // .withSupplyCurrentLimitEnable(true)
    // .withSupplyCurrentLimit(60)
    // .withStatorCurrentLimitEnable(true)
    // .withStatorCurrentLimit(60);

    MotionMagicExpoVoltage m_request;
//add beam break sensor and motors
    public Intake() {
        intake = new TalonFX(Constants.intakeID);

        intake.setInverted(true);
        
        // configIntake = intake.getConfigurator();

        // currentLimitsConfigs = new CurrentLimitsConfigs()
        //     .withSupplyCurrentLimitEnable(true)
        //     .withSupplyCurrentLimit(60)
        //     .withStatorCurrentLimitEnable(true)
        //     .withStatorCurrentLimit(60);

        // configIntake.apply(currentLimitsConfigs);

            // Slot0Configs configs = new Slot0Configs().withKS(0.25).withKV(0.12).withKA(0.01).withKP(0.01).withKD(0);
        // desiredPos = 0;
        
        intakeConfig = new TalonFXConfiguration();
    //    MotionMagicConfigs wristMM = intakeConfig.MotionMagic
    //         .withMotionMagicCruiseVelocity(100)
    //         .withMotionMagicExpo_kV(0.12)
    //         .withMotionMagicExpo_kA(0.1);

    //     intake.getConfigurator().apply(wristMM);
    //     intake.getConfigurator().apply(configs);
    //     // intake.getConfigurator().apply(currentConfigs);
        
        intakeBeamBreak = new DigitalInput(9);

        m_request = new MotionMagicExpoVoltage(0);
    }
    //beam break port 9

    @Override
    public void periodic() {
        beamBroken = !intakeBeamBreak.get();
        // intake.setPosition(50);
        SmartDashboard.putBoolean("intakeBB", intakeBeamBreak.get());
        SmartDashboard.putNumber("intakeMotor", intake.getPosition().getValueAsDouble());
        

    }

    public void setSpeed(double speed) {
        intake.set(speed);
    }

    public void setPos(double pos) {
        intake.setControl(m_request.withPosition(pos));
    }

    public boolean isBeamBroken() {
        return beamBroken;
    }
}
