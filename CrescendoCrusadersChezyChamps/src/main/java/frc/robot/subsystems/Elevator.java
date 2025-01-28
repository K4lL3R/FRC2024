package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private TalonFX elevatorL;
    private TalonFX elevatorR;
    private TalonFXConfiguration eleConfigsL;
    private TalonFXConfiguration eleConfigsR;
    public double desiredPos;
    public double desiredPosR;

    private static final CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs()
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(60)
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(60);

    public Elevator() {
        elevatorL = new TalonFX(Constants.ElevatorConstants.motorID1);
        elevatorR = new TalonFX(Constants.ElevatorConstants.motorID2);
        eleConfigsL = new TalonFXConfiguration();
        eleConfigsR = new TalonFXConfiguration();
        Slot0Configs slot0ConfigsL = eleConfigsL.Slot0.withKS(0.36).withKV(0.12).withKA(0.01).withKP(0.123).withKD(0.36)
        .withKG(0.56).withGravityType(GravityTypeValue.Elevator_Static);
        Slot0Configs slot0ConfigsR = eleConfigsR.Slot0.withKS(0.36).withKV(0.12).withKA(0.01).withKP(0.123).withKD(0.36)
        .withKG(0.56).withGravityType(GravityTypeValue.Elevator_Static);
        desiredPos = 0;
        desiredPosR = 0;

        MotionMagicConfigs PIDLeft = eleConfigsL.MotionMagic
            .withMotionMagicCruiseVelocity(90)
            .withMotionMagicExpo_kV(0.1)
            .withMotionMagicExpo_kA(0.035);
        MotionMagicConfigs PIDRight = eleConfigsR.MotionMagic
            .withMotionMagicCruiseVelocity(90)
            .withMotionMagicExpo_kV(0.1)
            .withMotionMagicExpo_kA(0.035);

        elevatorL.getConfigurator().apply(eleConfigsL);
        elevatorL.getConfigurator().apply(currentConfigs);
        elevatorR.getConfigurator().apply(eleConfigsR);
        elevatorR.getConfigurator().apply(currentConfigs);

        elevatorL.setNeutralMode(NeutralModeValue.Brake);
        elevatorR.setNeutralMode(NeutralModeValue.Brake);

        elevatorL.setPosition(0);
        elevatorR.setPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("LVelo", elevatorL.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("RVelo", elevatorR.getPosition().getValueAsDouble());
        // SmartDashboard.putNumber("LVolt", elevatorL.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("getEleSpeed", elevatorL.get());
    }

    public void setEleHeight(Constants.ElevatorConstants.ModesEle mode) {
        switch (mode) {
            case Amp:
                desiredPos = Constants.ElevatorConstants.EleSetpoints.ampHeight;
                desiredPosR = Constants.ElevatorConstants.EleSetpoints.ampHeightR;
                break;
            case Down:
                desiredPos = Constants.ElevatorConstants.EleSetpoints.downPos;
                desiredPosR = Constants.ElevatorConstants.EleSetpoints.downPosR;
                break;
            case Trap:
                desiredPos = Constants.ElevatorConstants.EleSetpoints.trapHeight;
                desiredPosR = Constants.ElevatorConstants.EleSetpoints.trapHeightR;
                break;
        }
        elevatorL.setControl(new MotionMagicExpoVoltage(desiredPos));
        elevatorR.setControl(new MotionMagicExpoVoltage(desiredPosR));

    }

    public double getEleLPos() {
        return elevatorL.getPosition().getValueAsDouble();
    }

    public double getEleRPos() {
        return elevatorR.getPosition().getValueAsDouble();
    }

    public double getEleSpeed() {
        return elevatorL.get();
    }

    public void stopMotors() {
        elevatorL.stopMotor();
        elevatorR.stopMotor();
    }

    public void elePower(double power) {
        elevatorL.set(power);
        elevatorR.set(-power);
    }
}
