package frc.robot.subsystems;

import frc.lib.LimelightHelpers;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
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
            wristMotor.setIdleMode(IdleMode.kBrake);

            intakeController = wristMotor.getPIDController();
            // intakeController.setP(0.0003);
            // intakeController.setD(0.000011);
            intakeController.setFF(0.00008);
            intakeController.setSmartMotionAllowedClosedLoopError(0.01, 0);
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

        }
    }

    public static class wristShooter extends SubsystemBase {
        public CANSparkMax wristShooter;
        public SparkPIDController shootController;
        public DutyCycleEncoder shootWristEncoder;
        public static double distance;
        //public static double[] points = {-7, -20.1, -24, -25, -28.1};//4 foot
        public static double[] points = {-7.33, -7.33, -11.69, -16.8, -19.5, -22.2, -22.8, -22.8, -24.4, -25
            , -26, -26.2, -26.4};//1 foot
        //teleop points 0 = -7.3, 4 = -20.9, 8 = -25.4, 12 = -27.4, 16 = -28.1
        public static double[] pointsAuto = {-7.33, -7.33, -11.69, -16.8, -19.5, -22.2, -22.8, -24.4, -24.8, -25.2
            , -25.2, -25.7, -25.7};
        //auto points 0 = -8, 4 = -20.7, 8 = 24.9, 12 = -26, 16 = -28.6
        // public static double[] distancePOI = {34.85, 77.9, 115, 163.27, 211.3}; shop
        // public static double[] distancePOIB = {45, 93.18, 139, 173.27, 221.3};//field
        //public static double[] distancePOI = {34.85, 77.9, 131, 183, 221.3};//fu
       // public static double[] distancePOI = {50, 113, 168, 240, 260}; //4 foot field
        // public static double[] distancePOI = {50.71, 66.5, 80.5, 98.3, 112.6, 128.2, 143.9, 158, 167.4, 181, 204.2, 223, 243.4}; //1 foot field
        //public static double[] distancePOI = {43, 91, 141, 183, 221}; //field
        //original 0-34.85, 77.9, 131, 183, 221.3
        //Blue 0-42.6, 4-91.5, 8-142, 12-196
        //Red 0-43.5, 4-91, 8-142.5, 12-193.8,
        //distances: 50.71, 66.5, 80.5, 98.3, 112.6, 128.2, 143.9, 158, 167.4, 181, 204.2, 223, 243.4
        //setpoints: -7.33, -7.33, -11.19, -14.62, -17.69, -20.12, -21.35, -22.5, -22.65, -24, -24, -24, -24
        public static double[] distancePOIShop = {38.01, 50.27, 60.3, 73.4, 83.19, 92.4, 102.1, 109.9, 115.7, 124, 138, 147.8, 162};
        public static double[] distancePOI = {42.33, 52.53, 64.18, 75.69, 87.73, 98.31, 108.97, 119.85, 130.11, 139.17, 149.03, 160.28, 170};
        public wristShooter() {
            wristShooter = new CANSparkMax(25, MotorType.kBrushless);
            wristShooter.setSmartCurrentLimit(40);
            wristShooter.enableVoltageCompensation(12);
            wristShooter.setIdleMode(IdleMode.kBrake);

            shootController = wristShooter.getPIDController();
            shootController.setFF(0.00004);//0.00009
            shootController.setSmartMotionAllowedClosedLoopError(0.01, 0);
            shootController.setSmartMotionMaxVelocity(50000, 0);
            shootController.setSmartMotionMaxAccel(100000, 0);

            shootWristEncoder = new DutyCycleEncoder(3);
        }

        public void wristToPositionShooter(Constants.Wrists.ShooterConst.ShooterMode shooter) {
            shootController.setSmartMotionMaxVelocity(500000, 0);
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
                break;
            }
        }

        @Override
        public void periodic() {
            SmartDashboard.putNumber("Wrist Shoot Encoder Value", shootWristEncoder.getAbsolutePosition());
            // SmartDashboard.putNumber("Wrist Non-Absolute", intakeWristEncoder.setDistancePerRotationDistancePerRotation());
            SmartDashboard.putNumber("Wrist Shoot Encoder Value nonAbs", wristShooter.getEncoder().getPosition());
            distance = getDistance();
            SmartDashboard.putNumber("Distance", distance);
            SmartDashboard.putNumber("Deg", ticksToDeg(wristShooter.getEncoder().getPosition()));
        }

        public static double ticksToDeg(double ticks) {
            return (ticks / 0.54) + 77.5;//71
        }

        public static double degToTicks(double deg) {
            return 0.54 * (deg - 77.5);//71
        }
        //75.56

         public static double getDistance() {
           return 34 / Math.tan((13 - LimelightHelpers.getTX("limelight")) * (Math.PI / 180));
         }
     

         public void setShooterAngle(double distance) {
            shootController.setSmartMotionMaxVelocity(3000, 0);//10000
           double shooterAngleInTicks = degToTicks((Math.atan(78/distance) * (180/Math.PI)));
           shootController.setReference(shooterAngleInTicks, ControlType.kSmartMotion);
         }

         public static double interpolate(double startpointAngle, double endpointAngle, double startpointDist, double endpointDist, double currentDist) {
            return startpointAngle + (((currentDist - startpointDist) * (endpointAngle - startpointAngle)) / (endpointDist - startpointDist));
         }
         
         public void interpolatedAngle(double distance) {
            // if (DriverStation.getAlliance().isPresent()) {
            //     if (DriverStation.getAlliance().get() == Alliance.Red) {
            //         if (distance >= distancePOI[0] && distance <= distancePOI[1]) {
            //             shootController.setReference(interpolate(points[0], points[1], distancePOI[0], distancePOI[1], distance), ControlType.kSmartMotion);
            //         } else if (distance > distancePOI[1] && distance <= distancePOI[2]) {
            //             shootController.setReference(interpolate(points[1], points[2], distancePOI[1], distancePOI[2], distance), ControlType.kSmartMotion);
            //         } else if (distance > distancePOI[2] && distance <= distancePOI[3]) {
            //             shootController.setReference(interpolate(points[2], points[3], distancePOI[2], distancePOI[3], distance), ControlType.kSmartMotion);
            //         } else if (distance > distancePOI[3] && distance <= distancePOI[4]) {
            //             shootController.setReference(interpolate(points[3], points[4], distancePOI[3], distancePOI[4], distance), ControlType.kSmartMotion);
            //         }
            //     } else {
            //         if (distance >= distancePOIBlue[0] && distance <= distancePOIBlue[1]) {
            //             shootController.setReference(interpolate(points[0], points[1], distancePOIBlue[0], distancePOIBlue[1], distance), ControlType.kSmartMotion);
            //         } else if (distance > distancePOIBlue[1] && distance <= distancePOIBlue[2]) {
            //             shootController.setReference(interpolate(points[1], points[2], distancePOIBlue[1], distancePOIBlue[2], distance), ControlType.kSmartMotion);
            //         } else if (distance > distancePOIBlue[2] && distance <= distancePOIBlue[3]) {
            //             shootController.setReference(interpolate(points[2], points[3], distancePOIBlue[2], distancePOIBlue[3], distance), ControlType.kSmartMotion);
            //         } else if (distance > distancePOIBlue[3] && distance <= distancePOIBlue[4]) {
            //             shootController.setReference(interpolate(points[3], points[4], distancePOIBlue[3], distancePOIBlue[4], distance), ControlType.kSmartMotion);
            //         }    
            //     }
            // }
            if (distance >= distancePOI[0] && distance <= distancePOI[1]) {
                shootController.setReference(interpolate(pointsAuto[0], pointsAuto[1], distancePOI[0], distancePOI[1], distance), ControlType.kSmartMotion);
            } else if (distance > distancePOI[1] && distance <= distancePOI[2]) {
                shootController.setReference(interpolate(pointsAuto[1], pointsAuto[2], distancePOI[1], distancePOI[2], distance), ControlType.kSmartMotion);
            } else if (distance > distancePOI[2] && distance <= distancePOI[3]) {
                shootController.setReference(interpolate(pointsAuto[2], pointsAuto[3], distancePOI[2], distancePOI[3], distance), ControlType.kSmartMotion);
            } else if (distance > distancePOI[3] && distance <= distancePOI[4]) {
                shootController.setReference(interpolate(pointsAuto[3], pointsAuto[4], distancePOI[3], distancePOI[4], distance), ControlType.kSmartMotion);
            } else if (distance > distancePOI[4] && distance <= distancePOI[5]) {
                shootController.setReference(interpolate(pointsAuto[4], pointsAuto[5], distancePOI[4], distancePOI[5], distance), ControlType.kSmartMotion);
            } else if (distance > distancePOI[5] && distance <= distancePOI[6]) {
                shootController.setReference(interpolate(pointsAuto[5], pointsAuto[6], distancePOI[5], distancePOI[6], distance), ControlType.kSmartMotion);
            } else if (distance > distancePOI[6] && distance <= distancePOI[7]) {
                shootController.setReference(interpolate(pointsAuto[6], pointsAuto[7], distancePOI[6], distancePOI[7], distance), ControlType.kSmartMotion);
            } else if (distance > distancePOI[7] && distance <= distancePOI[8]) {
                shootController.setReference(interpolate(pointsAuto[7], pointsAuto[8], distancePOI[7], distancePOI[8], distance), ControlType.kSmartMotion);
            } else if (distance > distancePOI[8] && distance <= distancePOI[9]) {
                shootController.setReference(interpolate(pointsAuto[8], pointsAuto[9], distancePOI[8], distancePOI[9], distance), ControlType.kSmartMotion);
            } else if (distance > distancePOI[9] && distance <= distancePOI[10]) {
                shootController.setReference(interpolate(pointsAuto[9], pointsAuto[10], distancePOI[9], distancePOI[10], distance), ControlType.kSmartMotion);
            } else if (distance > distancePOI[10] && distance <= distancePOI[11]) {
                shootController.setReference(interpolate(pointsAuto[10], pointsAuto[11], distancePOI[10], distancePOI[11], distance), ControlType.kSmartMotion);
            } else if (distance > distancePOI[11] && distance <= distancePOI[12]) {
                shootController.setReference(interpolate(pointsAuto[11], pointsAuto[12], distancePOI[11], distancePOI[12], distance), ControlType.kSmartMotion);
            }
         }

         public void interpolatedAngleAuto(double distance) {
            // if (DriverStation.getAlliance().isPresent()) {
            //     if (DriverStation.getAlliance().get() == Alliance.Red) {
            //         if (distance >= distancePOI[0] && distance <= distancePOI[1]) {
            //             shootController.setReference(interpolate(points[0], points[1], distancePOI[0], distancePOI[1], distance), ControlType.kSmartMotion);
            //         } else if (distance > distancePOI[1] && distance <= distancePOI[2]) {
            //             shootController.setReference(interpolate(points[1], points[2], distancePOI[1], distancePOI[2], distance), ControlType.kSmartMotion);
            //         } else if (distance > distancePOI[2] && distance <= distancePOI[3]) {
            //             shootController.setReference(interpolate(points[2], points[3], distancePOI[2], distancePOI[3], distance), ControlType.kSmartMotion);
            //         } else if (distance > distancePOI[3] && distance <= distancePOI[4]) {
            //             shootController.setReference(interpolate(points[3], points[4], distancePOI[3], distancePOI[4], distance), ControlType.kSmartMotion);
            //         }
            //     } else {
            //         if (distance >= distancePOIBlue[0] && distance <= distancePOIBlue[1]) {
            //             shootController.setReference(interpolate(points[0], points[1], distancePOIBlue[0], distancePOIBlue[1], distance), ControlType.kSmartMotion);
            //         } else if (distance > distancePOIBlue[1] && distance <= distancePOIBlue[2]) {
            //             shootController.setReference(interpolate(points[1], points[2], distancePOIBlue[1], distancePOIBlue[2], distance), ControlType.kSmartMotion);
            //         } else if (distance > distancePOIBlue[2] && distance <= distancePOIBlue[3]) {
            //             shootController.setReference(interpolate(points[2], points[3], distancePOIBlue[2], distancePOIBlue[3], distance), ControlType.kSmartMotion);
            //         } else if (distance > distancePOIBlue[3] && distance <= distancePOIBlue[4]) {
            //             shootController.setReference(interpolate(points[3], points[4], distancePOIBlue[3], distancePOIBlue[4], distance), ControlType.kSmartMotion);
            //         }    
            //     }
            // }
            if (distance >= distancePOI[0] && distance <= distancePOI[1]) {
                shootController.setReference(interpolate(points[0], points[1], distancePOI[0], distancePOI[1], distance), ControlType.kSmartMotion);
            } else if (distance > distancePOI[1] && distance <= distancePOI[2]) {
                shootController.setReference(interpolate(points[1], points[2], distancePOI[1], distancePOI[2], distance), ControlType.kSmartMotion);
            } else if (distance > distancePOI[2] && distance <= distancePOI[3]) {
                shootController.setReference(interpolate(points[2], points[3], distancePOI[2], distancePOI[3], distance), ControlType.kSmartMotion);
            } else if (distance > distancePOI[3] && distance <= distancePOI[4]) {
                shootController.setReference(interpolate(points[3], points[4], distancePOI[3], distancePOI[4], distance), ControlType.kSmartMotion);
            } else if (distance > distancePOI[4] && distance <= distancePOI[5]) {
                shootController.setReference(interpolate(points[4], points[5], distancePOI[4], distancePOI[5], distance), ControlType.kSmartMotion);
            } else if (distance > distancePOI[5] && distance <= distancePOI[6]) {
                shootController.setReference(interpolate(points[5], points[6], distancePOI[5], distancePOI[6], distance), ControlType.kSmartMotion);
            } else if (distance > distancePOI[6] && distance <= distancePOI[7]) {
                shootController.setReference(interpolate(points[6], points[7], distancePOI[6], distancePOI[7], distance), ControlType.kSmartMotion);
            } else if (distance > distancePOI[7] && distance <= distancePOI[8]) {
                shootController.setReference(interpolate(points[7], points[8], distancePOI[7], distancePOI[8], distance), ControlType.kSmartMotion);
            } else if (distance > distancePOI[8] && distance <= distancePOI[9]) {
                shootController.setReference(interpolate(points[8], points[9], distancePOI[8], distancePOI[9], distance), ControlType.kSmartMotion);
            } else if (distance > distancePOI[9] && distance <= distancePOI[10]) {
                shootController.setReference(interpolate(points[9], points[10], distancePOI[9], distancePOI[10], distance), ControlType.kSmartMotion);
            } else if (distance > distancePOI[10] && distance <= distancePOI[11]) {
                shootController.setReference(interpolate(points[10], points[11], distancePOI[10], distancePOI[11], distance), ControlType.kSmartMotion);
            } else if (distance > distancePOI[11] && distance <= distancePOI[12]) {
                shootController.setReference(interpolate(points[11], points[12], distancePOI[11], distancePOI[12], distance), ControlType.kSmartMotion);
            }
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
            wristMotorArm.setIdleMode(IdleMode.kBrake);
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
            // wristMotorArm.set(pidTest.calculate(wristMotorArm.getEncoder().getPosition(), pidTest.getSetpoint()));
        }

    }
}
//config and change position of wrist to down for intaking, aligned with shooter to feed into shooter
//regular upright stow position to feed into arm
//config and run motor to and from stow position to score in trap (wrist)