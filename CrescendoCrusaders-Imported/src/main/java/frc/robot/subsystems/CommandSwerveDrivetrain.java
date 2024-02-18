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
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.lib.ModifiedSignalLogger;
// import frc.lib.SwerveVoltageRequest;
import frc.robot.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

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

    // Creates a SysIdRoutine
    // private SwerveVoltageRequest driveRequest = new SwerveVoltageRequest(true);

    // SysIdRoutine routine = new SysIdRoutine(
    //     new SysIdRoutine.Config(null, null, null, ModifiedSignalLogger.logState()),
    //     new SysIdRoutine.Mechanism((Measure<Voltage> voltage) -> setControl(driveRequest.withVoltage(voltage.in(Units.Volts))), 
    //     null, 
    //     this)
    // );

    public final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
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
            3.91, this.getDriveBaseRadius(), 
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
        // NamedCommands.registerCommand("IntakeDown", RobotContainer.wristFeed);
        // NamedCommands.registerCommand("Intake Feed to Shooter", RobotContainer.wristDown);
        // NamedCommands.registerCommand("Spin Shooter", RobotContainer.spinShooter);
        // NamedCommands.registerCommand("Stop Shooter", RobotContainer.stopShooter);
        // NamedCommands.registerCommand("RunIntake", RobotContainer.intakeDisc);
        // NamedCommands.registerCommand("Stop Intake", RobotContainer.stopWristIntake);
        // NamedCommands.registerCommand("Feed Disc to Shooter", RobotContainer.outtakeDisc);
        // NamedCommands.registerCommand("ShootFromClose", RobotContainer.shootWristShooting);
        // NamedCommands.registerCommand("Shoot From Far", RobotContainer.shootFromFar);
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

        // applyRequest(() -> fieldCentricAngle.withTargetDirection(Rotation2d.fromDegrees(deg)));


    }

    @Override
    public void periodic() {
        for (int i = 0; i < 4; i++) {
            SmartDashboard.putNumber("DriveTempModule" + i + ": ", (RobotContainer.drivetrain.getModule(i).getDriveMotor().getDeviceTemp().getValue()) * (1.8) + 32);
            SmartDashboard.putNumber("AngleTempModule" + i + ": ", (RobotContainer.drivetrain.getModule(i).getSteerMotor().getDeviceTemp().getValue()) * (1.8) + 32);

        }
    }

    // public Command runDriveQuasiTest(SysIdRoutine.Direction direction)
    // {
    //     return routine.quasistatic(direction);
    // }

    // public Command runDriveDynamTest(SysIdRoutine.Direction direction) {
    //     return routine.dynamic(direction);
    // }
}
