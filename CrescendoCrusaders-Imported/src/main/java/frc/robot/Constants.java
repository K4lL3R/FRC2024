package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;
    
    public static final class Swerve {
        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

        public static final class SDSMK4i_Constants {
          public static final double wheelDiameter = Units.inchesToMeters(4.0);
          public static final double wheelCircumference = wheelDiameter * Math.PI;
          public static final double steerGearRatio = ((150.0 / 7.0) / 1.0);
          public static final boolean driveMotorInvert = true;
          public static final boolean steerMotorInvert = true;
          public static final class DriveRatios {
            public static final double SDSMK4i_L1 = (8.14 / 1.0);
            public static final double SDSMK4i_L2 = (6.75 / 1.0);
            public static final double SDSMK4i_L3 = (6.12 / 1.0);
          }
        }

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.848);
        public static final double wheelBase = Units.inchesToMeters(21.848);
        public static final double wheelCircumference = SDSMK4i_Constants.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = SDSMK4i_Constants.DriveRatios.SDSMK4i_L3;
        public static final double angleGearRatio = SDSMK4i_Constants.steerGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = SDSMK4i_Constants.steerMotorInvert;
        public static final boolean driveMotorInvert = SDSMK4i_Constants.driveMotorInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 40;
        public static final int drivePeakCurrentLimit = 70;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.3;
        public static final double angleKI = 0;
        public static final double angleKD = 0;
        public static final double angleKF = 0;

        public static final double driveAllowableError = 10.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output */
        public static final double driveKS = 0.01471;
        public static final double driveKV = 0.21372;
        public static final double driveKA = 0.03367;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5;
        /** Radians per Second */
        public static final double maxAngularVelocity = 9.5;

        /* Neutral Modes */
        public static final IdleMode angleIdleMode = IdleMode.kBrake;
        public static final IdleMode driveIdleMode = IdleMode.kBrake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class FL_Mod {
            public static final int angleMotorID = 1;
            public static final int driveMotorID = 2;
            public static final int canCoderID = 27;
            //Comp
            //public static final Rotation2d angleOffset = Rotation2d.fromDegrees(170.985);
            //Practice
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(172.650);

            /* Drive Motor PID Values */
            public static final double drivekP = 0.0002;
            public static final double drivekI = 0.0;
            public static final double drivekD = 0.002;
            public static final double drivekF = 0.00016;

            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, drivekP, drivekI, drivekD, drivekF);
        }

        /* Front Right Module - Module 1 */
        public static final class FR_Mod {
            public static final int angleMotorID = 6;
            public static final int driveMotorID = 5;
            public static final int canCoderID = 26;
            //Comp
            // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(257.560);
            //Practice
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(212.000);

            /* Drive Motor PID Values */
            public static final double drivekP = 0.0002;
            public static final double drivekI = 0.0;
            public static final double drivekD = 0.002;
            public static final double drivekF = 0.00016;

            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, drivekP, drivekI, drivekD, drivekF);
        }
        
        /* Back Left Module - Module 2 */
        public static final class BL_Mod {
            public static final int angleMotorID = 4;
            public static final int driveMotorID = 3;
            public static final int canCoderID = 24;
            //Comp
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(247.700);
            //Practice
            // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(131.780);

            /* Drive Motor PID Values */
            public static final double drivekP = 0.0002;
            public static final double drivekI = 0.0;
            public static final double drivekD = 0.002;
            public static final double drivekF = 0.00016;

            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, drivekP, drivekI, drivekD, drivekF);
        }

        /* Back Right Module - Module 3 */
        public static final class BR_Mod {
            public static final int angleMotorID = 7;
            public static final int driveMotorID = 8;
            public static final int canCoderID = 25;
            //Comp
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(127.800);
            //Practice
            //public static final Rotation2d angleOffset = Rotation2d.fromDegrees(346.660);
            
            /* Drive Motor PID Values */
            public static final double drivekP = 0.0001;
            public static final double drivekI = 0.0;
            public static final double drivekD = 0.001;
            public static final double drivekF = 0.00016;

            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, drivekP, drivekI, drivekD, drivekF);
        }
    }

    public static final class Elevator {
        public static final int canID = 10;
        public static final int encoderPort = 7;
        //Comp
        // public static final double mappingOffset = 0.410;
        // public static final double mappingHeight = 9.0;
        //Practice
        public static final double mappingOffset = 0.222;
        public static final double mappingHeight = 9.0;
        public static final IdleMode idleMode = IdleMode.kBrake;

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0.00005;
        public static final double maxVel = 24000;
        public static final double maxAccel = 96000;


        public static final class ConeSetpoints {
            public static final double high = 53.0;
            public static final double mid = 40.0;
        }
        public static final class CubeSetpoints {
            public static final double high = 48.0;
            public static final double mid = 24.0;
        }
        public static final class GlobalSetpoints {
            public static final double ret = 9.0;
            public static final double intake_cone = 15.0;
            public static final double intake_cube = 10.0;
            public static final double intake_cone_feeder = 10.0;
        }
    }

    public static final class Wrist {
        public static final int canID = 15;
        public static final int encoderPort = 0;
        public static final IdleMode idleMode = IdleMode.kBrake;
        public static final double hardStopDegOffset = 0.507;

        public static final class ConeSetpoints {
            public static final double high = 116.5;
            public static final double mid = 111.9;
        }
        public static final class CubeSetpoints {
            public static final double high = 180.0;
            public static final double mid = 175.0;
        }
        public static final class GlobalSetpoints {
            public static final double stow = 195.0;
            public static final double intake_cone = 100.174;
            public static final double intake_cube = 105.174;
            public static final double intake_cone_feeder = 180.0;
        }

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0.00008;
        public static final double maxVel = 24000;
        public static final double maxAccel = 24000;
    }

    public static final class Intake {
        public static final int canID = 9;
        public static final double intake_cone_outtake_cube = 1;//0.8
        public static final double intake_cube_outtake_cone = -1;//-0.5
    }
}