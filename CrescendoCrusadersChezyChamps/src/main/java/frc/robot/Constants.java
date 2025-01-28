// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;

import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int indexer1ID = 2;
    public static final int indexer2ID = 3;
    public static final int intakeID = 33;
    public static final int shooterLeftID = 30;
    public static final int shooterRightID = 31;
//l 0.49 r -0.40
//l 29 r -28.99
//l 26.66 r -26.64
    public final class ElevatorConstants {
        public static final int motorID1 = 35;
        public static final int motorID2 = 34;
        public class EleSetpoints {
          public static final double downPos = 0.42;
          public static final double ampHeight = 28.81;
          public static final double trapHeight = 26.46;
          public static final double downPosR = -0.6;
          public static final double ampHeightR = -28.99;
          public static final double trapHeightR = -26.64;
        }
        public enum ModesEle {
          Down,
          Amp,
          Trap
        }
    }

    public final class Climber {
      public static final int motorID1 = 41;
      public static final int motorID2 = 27;
      public class ClimbSetpoints {
        public static final double downPos = 35.26;
        public static final double climbingPos = -13.43;
        public static final double midPos = -8.7;
        public static final double downPosR = -37.12;
        public static final double climbingPosR = 14.14;
        public static final double midPosR = 9.3;
        public static final double zero = 0.0;
        public static final double zeroR = 0.0;
      }

      public enum Modes {
        Down,
        Climbing,
        Mid,
        Zero
      }

      //lPos = -13.43, -11.07, 35.26
      //rPos = 14.14, 11.67, -37.12
    }

    public final class AngleChanger {
      public static final int motorID = 32;
      public class ManualSetpoints {
        public static final double feedPos = -5.855;
        public static final double wooferPos = -4.5;
        public static final double backwardWooferPos = 4.5;
        public static final double closeTrussPos = -7.87;
        public static final double stowPos = 0;
        public static final double ampPos = 6.77;
        public static final double trapPos = -23.45;
        public static final double passPos = -4.85;
      }
      public enum Angles {
        Feed,
        Woofer,
        BackwardWoofer,
        CloseTruss,
        Stow,
        Amp,
        Trap,
        Pass
      }
      // public static final double[] points = {};
      // public static final double[] Constants.AngleChanger.distancearrayClose = {};
      // distances (bottom) 31.49;40.37;50.05;59.42;67.55;74.70;82.20;89.13
      //distance(top)121.65(8ft);131.66;150.96;165.28;183.60;193.28;203
      //distance: 38.72;46;57.9;69.95;81.13;91.56;103.39;114.64
      // points: -4.29;-5;-4.72;-5.87;-6.43;-6.79;-6.92;-7.08
      public static final double[] distancearrayClose = {31.49, 40.37, 50.05, 59.42, 67.55, 74.70, 82.20, 89.13};
      public static final double[] distancearrayFar = {121.65, 131.66, 150.96, 165.28, 183.60, 193.28, 203, 115.17};
      public static final double[] wristPointsClose = {-3.31, -4.22, -5.08, -5.69, -6.22, -6.29, -6.74, -6.74};
      public static final double[] wristPointsFar = {-7.14, -7.40, -7.57, -7.62, -7.92, -7.79, -7.93, -6.74};
      public static final InterpolatingDoubleTreeMap ANGLESINTERPOLATE_DOUBLE_TREE_MAP = new InterpolatingDoubleTreeMap();
      // static {
      //   // ANGLESINTERPOLATE_DOUBLE_TREE_MAP.put(31.49, null);
      //   // ANGLESINTERPOLATE_DOUBLE_TREE_MAP.put(null, null);
      // }
    }

    public final class TunerConstants {

      public enum DriveModes{
        FieldCentric,
        RobotCentric
      }
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.2)
        .withKS(0).withKV(1.5).withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(3).withKI(0).withKD(0)
        .withKS(0.26387).withKV(0.11656).withKA(0.0028061);
        //-0.088051, 0.11777, 0.021431

        

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double kSlipCurrentA = 150.0;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double kSpeedAt12VoltsMps = 5.95;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 0;//3.5714285714285716

    private static final double kDriveGearRatio = 5.36;
    private static final double kSteerGearRatio = 18.75;
    private static final double kWheelRadiusInches = 2;

    private static final boolean kSteerMotorReversed = true;
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final String kCANbusName = "";
    private static final int kPigeonId = 60;


    // These are only used for simulation
    private static final double kSteerInertia = 0.00001;
    private static final double kDriveInertia = 0.001;
    // Simulated voltage necessary to overcome friction
    private static final double kSteerFrictionVoltage = 0.25;
    private static final double kDriveFrictionVoltage = 0.25;

    private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withPigeon2Id(kPigeonId)
            .withCANbusName(kCANbusName);

    private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadiusInches)
            .withSlipCurrent(kSlipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage)
            .withFeedbackSource(SteerFeedbackType.RemoteCANcoder)
            .withCouplingGearRatio(kCoupleRatio)
            .withSteerMotorInverted(kSteerMotorReversed);


    // Front Left
    private static final int kFrontLeftDriveMotorId = 52;
    private static final int kFrontLeftSteerMotorId = 51;
    private static final int kFrontLeftEncoderId = 27;
    private static final double kFrontLeftEncoderOffset = 0.34521484375;//Robot2: 0.30224609375 Chezy: 

    private static final double kFrontLeftXPosInches = 12.25;
    private static final double kFrontLeftYPosInches = 11.75;

    // Front Right
    private static final int kFrontRightDriveMotorId = 53;
    private static final int kFrontRightSteerMotorId = 54;
    private static final int kFrontRightEncoderId = 26;
    private static final double kFrontRightEncoderOffset = 0.057861328125;//Robot2: 0.39208984375 Chezy: 

    private static final double kFrontRightXPosInches = 12.25;
    private static final double kFrontRightYPosInches = -11.75;

    // Back Left
    private static final int kBackLeftDriveMotorId = 57;
    private static final int kBackLeftSteerMotorId = 58;
    private static final int kBackLeftEncoderId = 24;
    private static final double kBackLeftEncoderOffset = -0.126220703125;//Robot2: -0.431884765625 Chezy: 

    private static final double kBackLeftXPosInches = -12.25;
    private static final double kBackLeftYPosInches = 11.75;

    // Back Right
    private static final int kBackRightDriveMotorId = 56;
    private static final int kBackRightSteerMotorId = 55;
    private static final int kBackRightEncoderId = 25;
    private static final double kBackRightEncoderOffset = 0.179443359375;//Robot2: -0.0263671875 Chezy: 

    private static final double kBackRightXPosInches = -12.25;
    private static final double kBackRightYPosInches = -11.75;


    private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
    private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
    private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
    private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);

    public static final CommandSwerveDrivetrain DriveTrain = new CommandSwerveDrivetrain(DrivetrainConstants, FrontLeft,
            FrontRight, BackLeft, BackRight);
}

}
