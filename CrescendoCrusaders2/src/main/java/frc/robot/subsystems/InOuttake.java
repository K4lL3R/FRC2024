package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class InOuttake {
    public static class Shooter extends SubsystemBase {
        public CANSparkMax shooterMotorUp;
        public CANSparkMax shooterMotorDown;
        public CANSparkMax shooterMotorHold;
        
        public Shooter() {
            shooterMotorUp = new CANSparkMax(44, MotorType.kBrushless);
            shooterMotorDown = new CANSparkMax(43, MotorType.kBrushless);
            shooterMotorHold = new CANSparkMax(42, MotorType.kBrushless);
            shooterMotorUp.enableVoltageCompensation(12);
            shooterMotorDown.enableVoltageCompensation(12);
            shooterMotorHold.enableVoltageCompensation(12);
            shooterMotorUp.setInverted(true);
            shooterMotorDown.setInverted(true);
            shooterMotorHold.setInverted(true);

        }

        public void runFlyWheels(double power) {
            shooterMotorUp.set(power);
            shooterMotorDown.set(power);
        }

        public void runFlyWheelsIdle(double power) {
            shooterMotorUp.set(power * 0.33);
            shooterMotorDown.set(power);
        }

        public void runHoldingMotor(double power) {
            shooterMotorHold.set(power);
        }
    }

    public static class ArmOuttake extends SubsystemBase {
        public CANSparkMax armOuttakeMotor;
        public static DigitalInput armBeamBreak;
        public static boolean beamBroken;

        public ArmOuttake() {
            armOuttakeMotor = new CANSparkMax(40, MotorType.kBrushless);
            armOuttakeMotor.setIdleMode(IdleMode.kBrake);

            armBeamBreak = new DigitalInput(8);
        }

        public void runArmOuttake(double power) {
            armOuttakeMotor.set(power);
            // if (armBeamBreak.get()) {
            //     armOuttakeMotor.stopMotor();
            // }
        }

        public boolean getIsBeamBrakeBroken() {
            return !armBeamBreak.get();
        }

        @Override
        public void periodic() {
            beamBroken = getIsBeamBrakeBroken();
            SmartDashboard.putBoolean("BeamBreak", armBeamBreak.get());
        }
    }

    public static class WristIntake extends SubsystemBase {
        public CANSparkMax wristIntakeMotor;
        public static DigitalInput wristBeamBreak;
        public static boolean beamBroken;

        public WristIntake() {

        wristIntakeMotor = new CANSparkMax(41, MotorType.kBrushless);
        wristIntakeMotor.setIdleMode(IdleMode.kBrake);
        wristIntakeMotor.setInverted(true);

        wristBeamBreak = new DigitalInput(9);
        
        }

        public void runWristIntake(double power) {
            wristIntakeMotor.set(power);
            // if (wristBeamBreak.get()) {
            //     wristIntakeMotor.stopMotor();
            // }
        }

        public boolean getIsBeamBrakeBroken() {
            return !wristBeamBreak.get();
        }

        @Override
        public void periodic() {
            beamBroken = getIsBeamBrakeBroken();
            SmartDashboard.putBoolean("BeamBreakWrist", wristBeamBreak.get());
        }
    }
}
//config and methods to run each intake/outtake motor
//outtake from arm for trap, run shooter, intake from wrist
//may need delay for shooter to spin up (here or in the sequences files)
//sensor to detect if note has been launched?