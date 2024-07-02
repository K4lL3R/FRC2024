package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.*;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    public Pigeon2 gyro;
    public FieldCentricFacingAngle fieldCentricAngle = new FieldCentricFacingAngle();
    Pose2d poseA;
    Pose2d poseB;
    StructPublisher<Pose2d> publisher;
    StructPublisher<Pose2d> publisher2;
    StructArrayPublisher<SwerveModuleState> publisher3;
    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
            m_kinematics, this.getPigeon2().getRotation2d(), m_modulePositions, new Pose2d());
    // LimelightHelpers.PoseEstimate llPoseEstimate;


    // Creates a SysIdRoutine
    // private SwerveVoltageRequest driveRequest = new SwerveVoltageRequest(true);

    // SysIdRoutine routine = new SysIdRoutine(
    //     new SysIdRoutine.Config(null, null, null, ModifiedSignalLogger.logState()),
    //     new SysIdRoutine.Mechanism((Measure<Voltage> voltage) -> setControl(driveRequest.withVoltage(voltage.in(Units.Volts))), 
    //     null, 
    //     this)
    // );

    public final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        publisher = NetworkTableInstance.getDefault()
            .getStructTopic("MyPose", Pose2d.struct).publish();
        publisher2 = NetworkTableInstance.getDefault()
            .getStructTopic("MyPoseAAA", Pose2d.struct).publish();
        publisher3 = NetworkTableInstance.getDefault()
            .getStructArrayTopic("Swerve", SwerveModuleState.struct).publish();
        this.seedFieldRelative(new Pose2d(1.58, 5.49, Rotation2d.fromDegrees(0)));
        LimelightHelpers.setCameraPose_RobotSpace("limelight", 0.3, 0, 0.1, 0, 40, 0);

        // llPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    }

    public double getDriveBaseRadius() {
        double driveRadius = 0;
        for (var modLocation : m_moduleLocations) {
            driveRadius = Math.max(driveRadius, modLocation.getNorm());
        }
        return driveRadius;
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public ChassisSpeeds getChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public void configPathPlanner() {
        AutoBuilder.configureHolonomic(
          () -> this.getState().Pose, // Pose2d supplier
          this::seedFieldRelative, // Pose2d consumer, used to reset odometry at the beginning of auto
          this::getChassisSpeeds,
          (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)),
          new HolonomicPathFollowerConfig(
            new PIDConstants(10, 0, 0), 
            new PIDConstants(7,0,0), 
            5.4, this.getDriveBaseRadius(), 
            new ReplanningConfig()),
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          }, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
          this // The drive subsystem. Used to properly set the requirements of path following commands
      );
    }

    public Command getAutonPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }


    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public void tare_Swerve(double deg) {
        TunerConstants.DriveTrain.getPigeon2().reset();
        TunerConstants.DriveTrain.getPigeon2().getConfigurator().setYaw(deg);

        // this.applyRequest(() -> fieldCentricAngle.withTargetDirection(Rotation2d.fromDegrees(deg)));


    }

    // public PathPlannerPath findPathToScore() {
    //     List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
    //         new Pose2d(this.getState().Pose.getX(), this.getState().Pose.getY(), Rotation2d.fromDegrees(160)),
    //         new Pose2d(2.54, 2.37, Rotation2d.fromDegrees(115.79))
    //     );

    //     PathPlannerPath path = new PathPlannerPath(
    //         bezierPoints, 
    //         new PathConstraints(3.0, 3.0, 3 * Math.PI, 4 * Math.PI), 
    //         new GoalEndState(0, Rotation2d.fromDegrees(-51.97)));
    //     return path;
    // }

    public Command findPath() {
        Pose2d targetPose = new Pose2d(2.54, 2.37, Rotation2d.fromDegrees(-51.97));
        PathConstraints constraints = new PathConstraints(3, 4, 3 * Math.PI, 4 * Math.PI);

        Command pathFind = AutoBuilder.pathfindToPose(targetPose, constraints, 0, 0);
        return pathFind;
    }

    @Override
    public void periodic() {
        updateOdometry();
        poseB = new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), poseEstimator.getEstimatedPosition().getRotation());
        poseA = new Pose2d(this.getState().Pose.getTranslation(), this.getState().Pose.getRotation());
        
        // SwerveDrivePoseEstimator estimator = new SwerveDrivePoseEstimator(m_kinematics, this.getPigeon2().getRotation2d(), m_modulePositions, poseB)
        // addVisionMeasurement(poseB, );
        for (int i = 0; i < 4; i++) {
            SmartDashboard.putNumber("DriveTempModule" + i + ": ", (RobotContainer.drivetrain.getModule(i).getDriveMotor().getDeviceTemp().getValue()) * (1.8) + 32);
            SmartDashboard.putNumber("AngleTempModule" + i + ": ", (RobotContainer.drivetrain.getModule(i).getSteerMotor().getDeviceTemp().getValue()) * (1.8) + 32);

        }
        SmartDashboard.putNumber("Rotation", this.getPigeon2().getRotation2d().getDegrees());

        // SmartDashboard.putString("Command", this.getCurrentCommand().toString());
        // SmartDashboard.putNumber("X", RobotContainer.controller.getX());
        // WPILib
        publisher2.set(poseA);
        publisher.set(poseB);
        // SmartDashboard.putNumber("Apriltags", llPoseEstimate.tagCount);
        // SmartDashboard.putNumber("GyroRate", this.getPigeon2().getRate());
    }

    // public Command runDriveQuasiTest(SysIdRoutine.Direction direction)
    // {
    //     return routine.quasistatic(direction);
    // }

    // public Command runDriveDynamTest(SysIdRoutine.Direction direction) {
    //     return routine.dynamic(direction);
    // }

    // private SwerveVoltageRequest driveVoltageRequest = new SwerveVoltageRequest(true);

    // private SysIdRoutine m_driveSysIdRoutine =
    // new SysIdRoutine(
    //     new SysIdRoutine.Config(null, null, null, ModifiedSignalLogger.logState()),
    //     new SysIdRoutine.Mechanism(
    //         (Measure<Voltage> volts) -> setControl(driveVoltageRequest.withVoltage(volts.in(Units.Volts))),
    //         null,
    //         this));

    // private SwerveVoltageRequest steerVoltageRequest = new SwerveVoltageRequest(false);

    // private SysIdRoutine m_steerSysIdRoutine =
    // new SysIdRoutine(
    //     new SysIdRoutine.Config(null, null, null, ModifiedSignalLogger.logState()),
    //     new SysIdRoutine.Mechanism(
    //         (Measure<Voltage> volts) -> setControl(steerVoltageRequest.withVoltage(volts.in(Units.Volts))),
    //         null,
    //         this));

    // private SysIdRoutine m_slipSysIdRoutine =
    // new SysIdRoutine(
    //     new SysIdRoutine.Config(Units.Volts.of(0.25).per(Units.Seconds.of(1)), null, null, ModifiedSignalLogger.logState()),
    //     new SysIdRoutine.Mechanism(
    //         (Measure<Voltage> volts) -> setControl(driveVoltageRequest.withVoltage(volts.in(Units.Volts))),
    //         null,
    //         this));

    // public Command runDriveQuasiTest(SysIdRoutine.Direction direction)
    // {
    //     return m_driveSysIdRoutine.quasistatic(direction);
    // }

    // public Command runDriveDynamTest(SysIdRoutine.Direction direction) {
    //     return m_driveSysIdRoutine.dynamic(direction);
    // }

    // public Command runSteerQuasiTest(SysIdRoutine.Direction direction)
    // {
    //     return m_steerSysIdRoutine.quasistatic(direction);
    // }

    // public Command runSteerDynamTest(SysIdRoutine.Direction direction) {
    //     return m_steerSysIdRoutine.dynamic(direction);
    // }

    // public Command runDriveSlipTest()
    // {
    //     return m_slipSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    // }

    public static double limelight_aim_proportional() {    
        // kP (constant of proportionality)
        // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
        // if it is too high, the robot will oscillate.
        // if it is too low, the robot will never reach its target
        // if the robot never turns in the correct direction, kP should be inverted.
            double kP = .01;
    
            // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
            // your limelight 3 feed, tx should return roughly 31 degrees.
            double targetingAngularVelocity = LimelightHelpers.getTY("limelight") * kP;
    
            // convert to radians per second for our drive method
            targetingAngularVelocity *= RobotContainer.MaxAngularRate;
    
            //invert since tx is positive when the target is to the right of the crosshair
            targetingAngularVelocity *= -1.0;
    
            return targetingAngularVelocity;
        }

        public void updateOdometry() {
            // LimelightHelpers.Results results = LimelightHelpers.getLatestResults("limelight").targetingResults;
            poseEstimator.update(this.getPigeon2().getRotation2d(), m_modulePositions);
            LimelightHelpers.SetRobotOrientation("limelight", poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
            LimelightHelpers.PoseEstimate llPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
               if (llPoseEstimate.tagCount > 0) {
                if (llPoseEstimate.rawFiducials[0].ambiguity <= 0.5 || llPoseEstimate.rawFiducials[0].distToCamera <= 6) {
                    poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(1, 1, 9999999));
                    // poseEstimator.addVisionMeasurement(llPoseEstimate.pose, llPoseEstimate.timestampSeconds);
                    poseEstimator.addVisionMeasurement(llPoseEstimate.pose, llPoseEstimate.timestampSeconds);
                    // System.out.print("hi");
                    // SmartDashboard.putNumber("distanceLL", llPoseEstimate.rawFiducials[0].distToCamera);
                    SmartDashboard.putNumber("ambiguity", llPoseEstimate.rawFiducials[0].ambiguity);
                    SmartDashboard.putNumber("distancetoCam", llPoseEstimate.rawFiducials[0].distToCamera);
                }
               }
            SmartDashboard.putNumber("Apriltag", llPoseEstimate.tagCount);
            SmartDashboard.putNumber("Rate", this.getPigeon2().getRate());
            SmartDashboard.putNumber("lengthLL", llPoseEstimate.rawFiducials.length);
            
            //    System.out.print("hey");
        }
    
    

}
