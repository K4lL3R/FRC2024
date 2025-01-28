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

public class ShooterWrist extends SubsystemBase {
    private TalonFX shooterWrist;
    private TalonFXConfiguration wristConfig;
    public double desiredPos;
    MotionMagicExpoVoltage m_request;
    private Vision visionLL;
    public double distance;
    public double distanceFar;
    private Slot0Configs configs;
    private MotionMagicConfigs wristMM;
    private double currentTimer;
    public boolean hasLostSight;

    private static final CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs()
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(60)
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(60);

    public ShooterWrist() {
        shooterWrist = new TalonFX(Constants.AngleChanger.motorID);
        shooterWrist.setNeutralMode(NeutralModeValue.Brake);
        shooterWrist.setInverted(true);

        configs = new Slot0Configs().withKS(0.465).withKV(0.12).withKA(0.01).withKP(0.4).withKD(0.37)
        .withKG(0.565).withGravityType(GravityTypeValue.Arm_Cosine);
        desiredPos = 0;
        //sigma alpha wolf programmer
        //awooooooo, wolves we roam the night
        
        wristConfig = new TalonFXConfiguration();
        wristMM = wristConfig.MotionMagic
            .withMotionMagicCruiseVelocity(130)
            .withMotionMagicExpo_kV(0.127)
            .withMotionMagicExpo_kA(0.15);

        

        shooterWrist.getConfigurator().apply(wristMM);
        shooterWrist.getConfigurator().apply(configs);
        shooterWrist.getConfigurator().apply(currentConfigs);
        m_request = new MotionMagicExpoVoltage(0).withEnableFOC(true);
        // for (double i: Constants.AngleChanger.distancearrayClose) {
        //     for (double j: Constants.AngleChanger.Constants.AngleChanger.wristPointsClose) {
        //         Constants.AngleChanger.ANGLESINTERPOLATE_DOUBLE_TREE_MAP.put(i, j);
        //     }
        // }

        // for (double i: Constants.AngleChanger.distancearrayFar) {
        //     for (double j: Constants.AngleChanger.wristPointsFar) {
        //         Constants.AngleChanger.ANGLESINTERPOLATE_DOUBLE_TREE_MAP.put(i, j);
        //     }
        // }
        visionLL = new Vision();
        distance = 0;
        distanceFar = 0;
    }

    @Override
    public void periodic() {
        distance = 
        // visionLL.getDistance() < Constants.AngleChanger.distancearrayClose[7] ? 
        Vision.getDistance();
        //  : visionLL.getDistance2();
        distanceFar = Vision.getDistance2();
        SmartDashboard.putNumber("wristRotations", shooterWrist.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("wristPos", getShooterPos());
        SmartDashboard.putNumber("WristSpeed", shooterWrist.get());
        SmartDashboard.putNumber("disNuts", distance);
        SmartDashboard.putBoolean("inPosition?", inPosition());
        SmartDashboard.putNumber("desiredPosWrist", desiredPos);
        // currentTimer = Timer.getFPGATimestamp();
        // hasLostSight = hasLostSight();
        // SmartDashboard.putBoolean("hasLostSight", hasLostSight);
    }

    public void setDesiredPos(Constants.AngleChanger.Angles wristPos) {
        shooterWrist.getConfigurator().apply(wristMM.withMotionMagicCruiseVelocity(130).withMotionMagicExpo_kA(0.15));
        switch (wristPos) {
            case Woofer:
            desiredPos = Constants.AngleChanger.ManualSetpoints.wooferPos;
                break;
            case Feed:
            desiredPos = Constants.AngleChanger.ManualSetpoints.feedPos;
                break;
            case BackwardWoofer: 
            desiredPos = Constants.AngleChanger.ManualSetpoints.backwardWooferPos;
                break;
            case CloseTruss:
            desiredPos = Constants.AngleChanger.ManualSetpoints.closeTrussPos;
                break;
            case Stow:
            desiredPos = Constants.AngleChanger.ManualSetpoints.stowPos;
                break;
            case Amp:
            desiredPos = Constants.AngleChanger.ManualSetpoints.ampPos;
                break;
            case Trap:
            desiredPos = Constants.AngleChanger.ManualSetpoints.trapPos;
                break;
            case Pass:
            desiredPos = Constants.AngleChanger.ManualSetpoints.passPos;
                break;
        }
        
        shooterWrist.setControl(new MotionMagicExpoVoltage(desiredPos).withEnableFOC(true));
    }

    public void spinToDesiredPos() {
        
    }

    public double getShooterPos() {
        return shooterWrist.getPosition().getValueAsDouble();
    }

    public double getWristSpeed() {
        return shooterWrist.get();
    }

    public void setWristSpeed(double speed) {
        shooterWrist.set(speed);
    }

    public void setWristTrackPos(double distance) {
        shooterWrist.getConfigurator().apply(wristMM.withMotionMagicCruiseVelocity(16).withMotionMagicExpo_kA(0.25));
        desiredPos = Constants.AngleChanger.ANGLESINTERPOLATE_DOUBLE_TREE_MAP.get(distance);
        shooterWrist.setControl(new MotionMagicExpoVoltage(desiredPos).withEnableFOC(true));
    }
    

    // public void interpolatedAngle(double distance) {
    //     // shooterWrist.getConfigurator().apply(configs.withKP(0.00001).withKD(0));
    //     shooterWrist.getConfigurator().apply(wristMM.withMotionMagicCruiseVelocity(25).withMotionMagicExpo_kA(0.065));
    //     if (distance >= Constants.AngleChanger.distancearrayClose[0] && distance <= Constants.AngleChanger.distancearrayClose[1]) {
    //        desiredPos = interpolate(Constants.AngleChanger.wristPointsClose[0], Constants.AngleChanger.wristPointsClose[1], Constants.AngleChanger.distancearrayClose[0], Constants.AngleChanger.distancearrayClose[1], distance);
    //     } else if (distance > Constants.AngleChanger.distancearrayClose[1] && distance <= Constants.AngleChanger.distancearrayClose[2]) {
    //        desiredPos = interpolate(Constants.AngleChanger.wristPointsClose[1], Constants.AngleChanger.wristPointsClose[2], Constants.AngleChanger.distancearrayClose[1], Constants.AngleChanger.distancearrayClose[2], distance);
    //     } else if (distance > Constants.AngleChanger.distancearrayClose[2] && distance <= Constants.AngleChanger.distancearrayClose[3]) {
    //        desiredPos = interpolate(Constants.AngleChanger.wristPointsClose[2], Constants.AngleChanger.wristPointsClose[3], Constants.AngleChanger.distancearrayClose[2], Constants.AngleChanger.distancearrayClose[3], distance);
    //     } else if (distance > Constants.AngleChanger.distancearrayClose[3] && distance <= Constants.AngleChanger.distancearrayClose[4]) {
    //        desiredPos = interpolate(Constants.AngleChanger.wristPointsClose[3], Constants.AngleChanger.wristPointsClose[4], Constants.AngleChanger.distancearrayClose[3], Constants.AngleChanger.distancearrayClose[4], distance);
    //     } else if (distance > Constants.AngleChanger.distancearrayClose[4] && distance <= Constants.AngleChanger.distancearrayClose[5]) {
    //        desiredPos = interpolate(Constants.AngleChanger.wristPointsClose[4], Constants.AngleChanger.wristPointsClose[5], Constants.AngleChanger.distancearrayClose[4], Constants.AngleChanger.distancearrayClose[5], distance);
    //     } else if (distance > Constants.AngleChanger.distancearrayClose[5] && distance <= Constants.AngleChanger.distancearrayClose[6]) {
    //        desiredPos = interpolate(Constants.AngleChanger.wristPointsClose[5], Constants.AngleChanger.wristPointsClose[6], Constants.AngleChanger.distancearrayClose[5], Constants.AngleChanger.distancearrayClose[6], distance);
    //     } else if (distance > Constants.AngleChanger.distancearrayClose[6] && distance <= Constants.AngleChanger.distancearrayClose[7]) {
    //        desiredPos = interpolate(Constants.AngleChanger.wristPointsClose[6], Constants.AngleChanger.wristPointsClose[7], Constants.AngleChanger.distancearrayClose[6], Constants.AngleChanger.distancearrayClose[7], distance);
    //     }
    //     shooterWrist.setControl(new MotionMagicExpoVoltage(desiredPos));
    //  }

    //  public void interpolatedAngleFar(double distance) {
    //     // shooterWrist.getConfigurator().apply(configs.withKP(0.00001).withKD(0));
    //     shooterWrist.getConfigurator().apply(wristMM.withMotionMagicCruiseVelocity(25).withMotionMagicExpo_kA(0.065));
    //     if (distance >= Constants.AngleChanger.distancearrayFar[0] && distance <= Constants.AngleChanger.distancearrayFar[1]) {
    //        desiredPos = interpolate(Constants.AngleChanger.wristPointsFar[0], Constants.AngleChanger.wristPointsFar[1], Constants.AngleChanger.distancearrayFar[0], Constants.AngleChanger.distancearrayFar[1], distance);
    //     } else if (distance > Constants.AngleChanger.distancearrayFar[1] && distance <= Constants.AngleChanger.distancearrayFar[2]) {
    //         desiredPos = interpolate(Constants.AngleChanger.wristPointsFar[1], Constants.AngleChanger.wristPointsFar[2], Constants.AngleChanger.distancearrayFar[1], Constants.AngleChanger.distancearrayFar[2], distance);
    //     } else if (distance > Constants.AngleChanger.distancearrayFar[2] && distance <= Constants.AngleChanger.distancearrayFar[3]) {
    //        desiredPos = interpolate(Constants.AngleChanger.wristPointsFar[2], Constants.AngleChanger.wristPointsFar[3], Constants.AngleChanger.distancearrayFar[2], Constants.AngleChanger.distancearrayFar[3], distance);
    //     } else if (distance > Constants.AngleChanger.distancearrayFar[3] && distance <= Constants.AngleChanger.distancearrayFar[4]) {
    //         desiredPos = interpolate(Constants.AngleChanger.wristPointsFar[3], Constants.AngleChanger.wristPointsFar[4], Constants.AngleChanger.distancearrayFar[3], Constants.AngleChanger.distancearrayFar[4], distance);
    //     } else if (distance > Constants.AngleChanger.distancearrayFar[4] && distance <= Constants.AngleChanger.distancearrayFar[5]) {
    //         desiredPos = interpolate(Constants.AngleChanger.wristPointsFar[4], Constants.AngleChanger.wristPointsFar[5], Constants.AngleChanger.distancearrayFar[4], Constants.AngleChanger.distancearrayFar[5], distance);
    //     } else if (distance > Constants.AngleChanger.distancearrayFar[5] && distance <= Constants.AngleChanger.distancearrayFar[6]) {
    //         desiredPos = interpolate(Constants.AngleChanger.wristPointsFar[5], Constants.AngleChanger.wristPointsFar[6], Constants.AngleChanger.distancearrayFar[5], Constants.AngleChanger.distancearrayFar[6], distance);
    //     } else if (distance > Constants.AngleChanger.distancearrayFar[7] && distance <= Constants.AngleChanger.distancearrayFar[0]) {
    //         desiredPos = interpolate(Constants.AngleChanger.wristPointsFar[7], Constants.AngleChanger.wristPointsFar[0], Constants.AngleChanger.distancearrayFar[7], Constants.AngleChanger.distancearrayFar[0], distance);
    //     }
    //     shooterWrist.setControl(new MotionMagicExpoVoltage(desiredPos));
    //  }

    //  public static double interpolate(double startpointAngle, double endpointAngle, double startpointDist, double endpointDist, double currentDist) {
    //     return startpointAngle + (((currentDist - startpointDist) * (endpointAngle - startpointAngle)) / (endpointDist - startpointDist));
    //  }

     public boolean inPosition(){
        if (shooterWrist.getPosition().getValueAsDouble() > desiredPos - 0.15 && shooterWrist.getPosition().getValueAsDouble() < desiredPos + 0.15) {
            return true;
        }
        return false;
    }

    public void wristSetControl(double desiredPos) {
        shooterWrist.setControl(new MotionMagicExpoVoltage(desiredPos));
    }

    // public boolean hasLostSight() {
    //     // if (LimelightHelpers.getFiducialID("limelight-upfront") != -1) {
    //     //     Timer.delay(0.2);
    //     //     if (Timer.getFPGATimestamp() < currentTimer + 0.1){
    //     //         if (LimelightHelpers.getFiducialID("limelight-upfront") == -1) {
    //     //             // desiredPos = Constants.AngleChanger.ANGLESINTERPOLATE_DOUBLE_TREE_MAP.get(distanceFar);
    //     //             // shooterWrist.setControl(new MotionMagicExpoVoltage(desiredPos));
    //     //             return true;
    //     //         }
    //     //     }            
    //     // }
    //     // return false;
        
    // }
    
}
