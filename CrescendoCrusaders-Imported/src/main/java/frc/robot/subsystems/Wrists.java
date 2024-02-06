package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrists {

    public static double wristConvDegtoRot(double deg) {
        return ((deg/360.0) * 78.125);
    }

    public double wristConvRottoDeg(double rot) {
        return ((rot / (78.125)) * 360.0);
    }

    public static class wristIntake extends SubsystemBase {
        public CANSparkMax wristMotorIntake;
        public SparkPIDController intakeController;
        public DutyCycleEncoder intakeWristEncoder;



        public wristIntake() {
            wristMotorIntake = new CANSparkMax(27, MotorType.kBrushless);

            intakeController = wristMotorIntake.getPIDController();
            intakeController.setFF(0.0016);
            wristToPositionIntake(Constants.Wrists.Intake.IntakeMode.Stow);

            intakeWristEncoder = new DutyCycleEncoder(0);
        }

        public void wristToPositionIntake(Constants.Wrists.Intake.IntakeMode intake) {
            switch (intake) {
                case Down: intakeController.setReference(Constants.Wrists.Intake.intakeSetPoint, ControlType.kSmartMotion);
                    break;
                case Stow: intakeController.setReference(Constants.Wrists.Intake.stowSetPoint, ControlType.kSmartMotion);
                    break;
                case Feed: intakeController.setReference(Constants.Wrists.Intake.feedSetPoint, ControlType.kSmartMotion);
                    break;
            }
        }

        @Override
        public void periodic() {
            // TODO Auto-generated method stub
            SmartDashboard.putNumber("Wrist Encoder Value", intakeWristEncoder.getAbsolutePosition());
            // SmartDashboard.putNumber("Wrist Non-Absolute", intakeWristEncoder.setDistancePerRotationDistancePerRotation());
            SmartDashboard.putNumber("Wrist Encoder Value nonAbs", wristMotorIntake.getEncoder().getPosition());
        }
    }

    public static class wristShooter extends SubsystemBase {
        public CANSparkMax wristShooter;
        public SparkPIDController shootController;
        public DutyCycleEncoder shootWristEncoder;

        public wristShooter() {
            wristShooter = new CANSparkMax(25, MotorType.kBrushless);

            shootController = wristShooter.getPIDController();
            shootController.setFF(0.0016);
            wristToPositionShooter(Constants.Wrists.Shooter.ShooterMode.Close);

            shootWristEncoder = new DutyCycleEncoder(3);
        }

        public void wristToPositionShooter(Constants.Wrists.Shooter.ShooterMode shooter) {
            switch (shooter) {
                case Close: shootController.setReference(Constants.Wrists.Shooter.closeSetPoint, ControlType.kSmartMotion);
                break;
                case Down: shootController.setReference(Constants.Wrists.Shooter.downSetPoint, ControlType.kSmartMotion);
                break;
            }
        }

        @Override
        public void periodic() {
            // TODO Auto-generated method stub
            SmartDashboard.putNumber("Wrist Shoot Encoder Value", shootWristEncoder.getAbsolutePosition());
            // SmartDashboard.putNumber("Wrist Non-Absolute", intakeWristEncoder.setDistancePerRotationDistancePerRotation());
            SmartDashboard.putNumber("Wrist Shoot Encoder Value nonAbs", wristShooter.getEncoder().getPosition());
        }
    }

    public static class armWrist extends SubsystemBase {
        public CANSparkMax wristMotorArm;
        public SparkPIDController armController;

        public DutyCycleEncoder armEncoder;

        public armWrist() {

            wristMotorArm = new CANSparkMax(26, MotorType.kBrushless);
            wristMotorArm.setSmartCurrentLimit(40);
            wristMotorArm.enableVoltageCompensation(12);
            armController = wristMotorArm.getPIDController();
            armController.setP(0.00);
            armController.setD(0.00);
            // armController.setFF(0.00019);
            armController.setSmartMotionAllowedClosedLoopError(0.8, 0);
            armController.setSmartMotionMaxVelocity(500000, 0);
            armController.setSmartMotionMaxAccel(100000, 0);

            // wristToPositionArm(Constants.Wrists.Arm.ArmMode.Stow);

            armEncoder = new DutyCycleEncoder(1);
        }

        public void wristToPositionArm(Constants.Wrists.Arm.ArmMode arm) {
            switch (arm) {
                case Stow: armController.setReference(Constants.Wrists.Arm.stowSetPoint, ControlType.kSmartMotion);
                    break;
                case Score: armController.setReference(Constants.Wrists.Arm.scoreSetPoint, ControlType.kSmartMotion);
                    break;
            }
        }
        @Override
        public void periodic() {
            // TODO Auto-generated method stub
            SmartDashboard.putNumber("Wrist Arm Encoder Value", armEncoder.getAbsolutePosition());
            // SmartDashboard.putNumber("Wrist Non-Absolute", intakeWristEncoder.setDistancePerRotationDistancePerRotation());
            SmartDashboard.putNumber("Wrist Arm Encoder Value nonAbs", wristMotorArm.getEncoder().getPosition());
            SmartDashboard.putNumber("Arm", wristMotorArm.getEncoder().getPosition());
            SmartDashboard.putNumber("Volt", wristMotorArm.getEncoder().getVelocity());
        }
    }
}
//config and change position of wrist to down for intaking, aligned with shooter to feed into shooter
//regular upright stow position to feed into arm
//config and run motor to and from stow position to score in trap (wrist)