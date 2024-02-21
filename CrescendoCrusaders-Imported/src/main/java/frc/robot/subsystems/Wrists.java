package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
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

    public static class Wrist extends SubsystemBase {
        public CANSparkMax wristMotor;
        public SparkPIDController intakeController;
        public DutyCycleEncoder WristEncoder;



        public Wrist() {
            wristMotor = new CANSparkMax(27, MotorType.kBrushless);
            wristMotor.setSmartCurrentLimit(40);
            wristMotor.enableVoltageCompensation(12);

            intakeController = wristMotor.getPIDController();
            intakeController.setFF(0.0016);
            intakeController.setSmartMotionAllowedClosedLoopError(0.8, 0);
            intakeController.setSmartMotionMaxVelocity(500000, 0);
            intakeController.setSmartMotionMaxAccel(100000, 0);

            WristEncoder = new DutyCycleEncoder(0);
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
            SmartDashboard.putNumber("Wrist Encoder Value", WristEncoder.getAbsolutePosition());
            // SmartDashboard.putNumber("Wrist Non-Absolute", intakeWristEncoder.setDistancePerRotationDistancePerRotation());
            SmartDashboard.putNumber("Wrist Encoder Value nonAbs", wristMotor.getEncoder().getPosition());

        }
    }

    public static class wristShooter extends SubsystemBase {
        public CANSparkMax wristShooter;
        public SparkPIDController shootController;
        public DutyCycleEncoder shootWristEncoder;

        public wristShooter() {
            wristShooter = new CANSparkMax(25, MotorType.kBrushless);
            wristShooter.setSmartCurrentLimit(40);
            wristShooter.enableVoltageCompensation(12);

            shootController = wristShooter.getPIDController();
            shootController.setFF(0.0015);
            shootController.setSmartMotionAllowedClosedLoopError(0.8, 0);
            shootController.setSmartMotionMaxVelocity(500000, 0);
            shootController.setSmartMotionMaxAccel(100000, 0);

            shootWristEncoder = new DutyCycleEncoder(3);
        }

        public void wristToPositionShooter(Constants.Wrists.ShooterConst.ShooterMode shooter) {
            switch (shooter) {
                case ClimbLock: shootController.setReference(Constants.Wrists.ShooterConst.climbLock, ControlType.kSmartMotion);
                break;
                case Down: shootController.setReference(Constants.Wrists.ShooterConst.downSetPoint, ControlType.kSmartMotion);
                break;
                case Shooting: shootController.setReference(Constants.Wrists.ShooterConst.shooterSetPoint, ControlType.kSmartMotion);
                break;
                case FarShots: shootController.setReference(Constants.Wrists.ShooterConst.farShots, ControlType.kSmartMotion);
                break;
                case AutoFar: shootController.setReference(Constants.Wrists.ShooterConst.autoFar, ControlType.kSmartMotion);
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

        public PIDController pidTest;

        public armWrist() {

            wristMotorArm = new CANSparkMax(20, MotorType.kBrushless);
            wristMotorArm.setSmartCurrentLimit(40);
            wristMotorArm.enableVoltageCompensation(12);
            armController = wristMotorArm.getPIDController();
            // armController.setP(0.00006);
            // armController.setD(0.0003);
            armController.setFF(0.00013);
            armController.setSmartMotionAllowedClosedLoopError(0.8, 0);
            armController.setSmartMotionMaxVelocity(500000, 0);
            armController.setSmartMotionMaxAccel(100000, 0);
            pidTest = new PIDController(0.00006, 0, 0.0003);
            pidTest.setSetpoint(Constants.Wrists.Arm.stowSetPoint);

            // wristToPositionArm(Constants.Wrists.Arm.ArmMode.Stow);

            armEncoder = new DutyCycleEncoder(1);
        }

        public void wristToPositionArm(Constants.Wrists.Arm.ArmMode arm) {
            switch (arm) {
                case Stow: armController.setReference(Constants.Wrists.Arm.stowSetPoint, ControlType.kSmartMotion);
                    break;
                case Score: armController.setReference(Constants.Wrists.Arm.scoreSetPoint, ControlType.kSmartMotion);
                    break;
                case Feed: armController.setReference(Constants.Wrists.Arm.feedSetPoint, ControlType.kSmartMotion);
                    break;
            }
        }

        public void setSetpoint(Constants.Wrists.Arm.ArmMode arm) {
            switch (arm) {
                case Stow: pidTest.setSetpoint(Constants.Wrists.Arm.stowSetPoint);
                    break;
                case Score: pidTest.setSetpoint(Constants.Wrists.Arm.scoreSetPoint);
                    break;
                case Feed: pidTest.setSetpoint(Constants.Wrists.Arm.feedSetPoint);
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
            SmartDashboard.putBoolean("getName()", armEncoder.isConnected());
            
            // wristMotorArm.set(pidTest.calculate(wristMotorArm.getEncoder().getPosition(), pidTest.getSetpoint()));
        }

    }
}
//config and change position of wrist to down for intaking, aligned with shooter to feed into shooter
//regular upright stow position to feed into arm
//config and run motor to and from stow position to score in trap (wrist)