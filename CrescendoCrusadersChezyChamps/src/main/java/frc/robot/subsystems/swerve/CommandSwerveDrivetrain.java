package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.TunerConstants;
import frc.robot.subsystems.Vision;
import frc.robot.LimelightHelpers;
import frc.robot.ModifiedSignalLogger;
import frc.robot.RobotContainer;
import frc.robot.SwerveVoltageRequest;

public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem{


    StructPublisher<Pose2d> publisher;
    Pose2d poseA;
    private Vision visionLL;
          //fieldcentric


    private final SwerveDrivePoseEstimator poseEstimator;
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(RobotContainer.MaxSpeed * 0.05).withRotationalDeadband(RobotContainer.MaxAngularRate * 0.03) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  //robotcentric for tracking
    public static final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
      .withDeadband(RobotContainer.MaxSpeed * 0.05).withRotationalDeadband(RobotContainer.MaxAngularRate * 0.03) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    public final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

        private SwerveVoltageRequest driveRequest = new SwerveVoltageRequest(true);

    SysIdRoutine routine = new SysIdRoutine(
        new SysIdRoutine.Config(null, null, null, ModifiedSignalLogger.logState()),
        new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> setControl(driveRequest.withVoltage(volts.in(Units.Volt))), null, this));

    private static final CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs()
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(60)
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(60);

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants constants, SwerveModuleConstants... constants2) {
        super(constants, constants2);

            publisher = NetworkTableInstance.getDefault()
            .getStructTopic("MyPose", Pose2d.struct).publish();
            this.getModule(0).getDriveMotor().getConfigurator().apply(currentConfigs);
            this.getModule(0).getSteerMotor().getConfigurator().apply(currentConfigs);
            this.getModule(1).getDriveMotor().getConfigurator().apply(currentConfigs);
            this.getModule(1).getSteerMotor().getConfigurator().apply(currentConfigs); 
            this.getModule(2).getDriveMotor().getConfigurator().apply(currentConfigs);
            this.getModule(2).getSteerMotor().getConfigurator().apply(currentConfigs); 
            this.getModule(3).getDriveMotor().getConfigurator().apply(currentConfigs);
            this.getModule(3).getSteerMotor().getConfigurator().apply(currentConfigs);
            this.configNeutralMode(NeutralModeValue.Brake);  

            poseEstimator = new SwerveDrivePoseEstimator(
            m_kinematics, this.getPigeon2().getRotation2d(), m_modulePositions, new Pose2d());

            visionLL = new Vision();

            configPathPlanner();


    }
    
    public void configPathPlanner() {
        AutoBuilder.configureHolonomic(
          () -> this.getState().Pose, // Pose2d supplier
          this::seedFieldRelative, // Pose2d consumer, used to reset odometry at the beginning of auto
          this::getChassisSpeeds,
          (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)),
          new HolonomicPathFollowerConfig(
            new PIDConstants(2.8, 0, 0), 
            new PIDConstants(5,0,0.1), 
            5.4, this.getDriveBaseRadius(), 
            new ReplanningConfig()),
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                // return alliance.get() == DriverStation.Alliance.Red;
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          }, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
          this // The drive subsystem. Used to properly set the requirements of path following commands
      );
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

    public Command getAutonPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public Command setSwerveState(Constants.TunerConstants.DriveModes mode, CommandJoystick controller) {
        switch (mode) {
            default:
                return this.applyRequest(() -> drive.withVelocityX(-MathUtil.applyDeadband(controller.getY(), 0.1) * RobotContainer.MaxSpeed) // Drive forward with
                // negative Y (forward)
                  .withVelocityY(-MathUtil.applyDeadband(controller.getX(), 0.1) * RobotContainer.MaxSpeed) // Drive left with negative X (left)
                  .withRotationalRate(-MathUtil.applyDeadband(controller.getRawAxis(4), 0.1) * RobotContainer.MaxAngularRate)); // Drive counterclockwise with negative X (left))
               
        
            case RobotCentric:
                return this.applyRequest(() -> driveRobotCentric.withVelocityX(-MathUtil.applyDeadband(controller.getY(), 0.1) * RobotContainer.MaxSpeed) // Drive forward with
                // negative Y (forward)
                  .withVelocityY(-MathUtil.applyDeadband(controller.getX(), 0.1) * RobotContainer.MaxSpeed) // Drive left with negative X (left)
                  .withRotationalRate(Vision.limelight_aim_proportional())); // Drive counterclockwise with negative X (left))
        }   
    }

    @Override
    public void periodic() {
        // if (!Auton.is4Note) {
            updateOdometry();
        // }
        poseA = new Pose2d(this.getState().Pose.getTranslation(), this.getState().Pose.getRotation());
        publisher.set(poseA);
        SmartDashboard.putNumber("swerve", this.getModule(0).getDriveMotor().getPosition().getValueAsDouble());

        SmartDashboard.putNumber("distance to speaker", this.getState().Pose.getTranslation().getDistance(new Translation2d(0, 5.5)));
        SmartDashboard.putNumber("angle", Math.acos(this.getState().Pose.getX() / this.getState().Pose.getTranslation().getDistance(new Translation2d(0, 5.5))) * (180 / Math.PI));
        SmartDashboard.putNumber("angle using translation", Math.acos(this.getState().Pose.getTranslation().getX() / this.getState().Pose.getTranslation().getDistance(new Translation2d(0, 5.5))) * (Math.PI));
        SmartDashboard.putNumber("PigeonAngle", this.getPigeon2().getAngle());
        double oppositeSide = this.getState().Pose.getY();
        double adjacentSide = this.getState().Pose.getX();
        SmartDashboard.putNumber("oSide", oppositeSide);
        SmartDashboard.putNumber("aSide", adjacentSide);
        SmartDashboard.putNumber("angle", angleToCorner());
    }
   
    public Command runDriveQuasiTest(SysIdRoutine.Direction direction)
    {
        return routine.quasistatic(direction);
    }

    public Command runDriveDynamTest(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

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
    //     new SysIdRoutine.Config(Voltage.of(0.25).per(Units.Sec.of(1)), null, null, ModifiedSignalLogger.logState()),
    //     new SysIdRoutine.Mechanism(
    //         (Measure<Voltage> volts) -> setControl(driveVoltageRequest.withVoltage(volts.in(Units.Volts))),
    //         null,
    //         this));


    // // public Command runSteerQuasiTest(SysIdRoutine.Direction direction)
    // // {
    // //     return m_steerSysIdRoutine.quasistatic(direction);
    // // }

    // // public Command runSteerDynamTest(SysIdRoutine.Direction direction) {
    // //     return m_steerSysIdRoutine.dynamic(direction);
    // // }

    // public Command runDriveSlipTest()
    // {
    //     return m_slipSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    // }

    public void tare_Swerve(double deg) {
        // TunerConstants.DriveTrain.getPigeon2().reset();
        this.getPigeon2().getConfigurator().setYaw(deg);

        // this.applyRequest(() -> fieldCentricAngle.withTargetDirection(Rotation2d.fromDegrees(deg)));


    }

        public void updateOdometry() {
            // LimelightHelpers.Results results = LimelightHelpers.getLatestResults("limelight").targetingResults;
            poseEstimator.update(this.getPigeon2().getRotation2d(), m_modulePositions);
            LimelightHelpers.SetRobotOrientation("limelight-upfront", this.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
            // LimelightHelpers.SetRobotOrientation("limelight-dfront", this.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
            LimelightHelpers.PoseEstimate llPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-upfront");
            // LimelightHelpers.PoseEstimate llPoseEstimate2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-dfront");
            if (this.getPigeon2().getRate() < 720 && llPoseEstimate.tagCount > 0) {
                // if (llPoseEstimate2.tagCount > 0) {
                //     this.setVisionMeasurementStdDevs(VecBuilder.fill(1.15, 1.15, 9999999));
                //     this.addVisionMeasurement(llPoseEstimate2.pose, llPoseEstimate2.timestampSeconds);
            //    } else if (llPoseEstimate.tagCount > 0) {
                    this.setVisionMeasurementStdDevs(VecBuilder.fill(1.15, 1.15, 9999999));
                    this.addVisionMeasurement(llPoseEstimate.pose, llPoseEstimate.timestampSeconds);
            //    } else if (llPoseEstimate2.tagCount > 0 && llPoseEstimate.tagCount > 0) {
            //         this.setVisionMeasurementStdDevs(VecBuilder.fill(1.15, 1.15, 9999999));
            //         this.addVisionMeasurement(llPoseEstimate.pose, llPoseEstimate.timestampSeconds);
            //         this.addVisionMeasurement(llPoseEstimate2.pose, llPoseEstimate2.timestampSeconds);
            //    }
                // }
            }
               
            // SmartDashboard.putNumber("Apriltag", llPoseEstimate.tagCount);
            // SmartDashboard.putNumber("Rate", this.getPigeon2().getRate());
            // SmartDashboard.putNumber("lengthLL", llPoseEstimate.rawFiducials.length);
            
            //    System.out.print("hey");
        }

        public double turnToSpeaker() {
            double speakerXBlue = 0.0;
            // double speakerXRed = 16.54;
            double speakerY = 5.55;
            double kP = 0.09;

            Translation2d speakerBluePos = new Translation2d(speakerXBlue, speakerY);
            double distance = this.getState().Pose.getTranslation().getDistance(speakerBluePos);
            double angle = 0;
            try {
                angle = Math.acos(this.getState().Pose.getTranslation().getX() / distance) * (180 / Math.PI);
            } catch (Exception e) {
                angle = 0.1;
            }
            
            
            return kP * angle * RobotContainer.MaxAngularRate;
        }

        public double angleToCorner() {
            double oppositeSide = 7.35 - this.getState().Pose.getTranslation().getY();
            double adjacentSide = this.getState().Pose.getTranslation().getX();
            return Math.atan((oppositeSide / adjacentSide));
        }
}
