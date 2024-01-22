package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.lib.Conversions;
import frc.lib.SwerveModuleLib;
import frc.lib.SwerveModuleConstants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;


public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    public CANSparkMax mAngleMotor;
    public SparkPIDController anglePIDController;
    public CANSparkMax mDriveMotor;
    public SparkPIDController drivePIDController;

    public CANcoder angleEncoder;
    public SwerveModuleConstants modConstants;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.modConstants = moduleConstants;
        this.angleOffset = modConstants.angleOffset;
        
        angleEncoder = new CANcoder(moduleConstants.canCoderID);
        configAngleEncoder();

        mAngleMotor = new CANSparkMax(modConstants.steerMotorID, MotorType.kBrushless);
        configAngleMotor();

        mDriveMotor = new CANSparkMax(modConstants.driveMotorID, MotorType.kBrushless);
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = SwerveModuleLib.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(percentOutput);
        }
        else {
            double velocity = Conversions.MPSToNeo(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            drivePIDController.setReference(velocity, ControlType.kVelocity, 0, feedforward.calculate(desiredState.speedMetersPerSecond), ArbFFUnits.kVoltage);
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.05)) ? lastAngle : desiredState.angle;
        anglePIDController.setReference(Conversions.degreesToNeo(angle.getDegrees(), Constants.Swerve.angleGearRatio), ControlType.kPosition);
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.neoToDegrees(mAngleMotor.getEncoder().getPosition(), Constants.Swerve.angleGearRatio));
    }

    public double encoderToDeg() {
        return ((angleEncoder.getAbsolutePosition()).getValue() * 360);
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(encoderToDeg());
    }

    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToNeo(encoderToDeg() - angleOffset.getDegrees(), Constants.Swerve.angleGearRatio);
        mAngleMotor.getEncoder().setPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        angleEncoder.getConfigurator().apply(encoderConfig);
        // angleEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        // angleEncoder.withSensorDirection(false);
        // angleEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    }

    private void configAngleMotor(){
        mAngleMotor.restoreFactoryDefaults();
        mAngleMotor.enableVoltageCompensation(12);
        mAngleMotor.setSmartCurrentLimit(Constants.Swerve.anglePeakCurrentLimit, Constants.Swerve.angleContinuousCurrentLimit);
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        mAngleMotor.setIdleMode(Constants.Swerve.angleIdleMode);
        anglePIDController = mAngleMotor.getPIDController();
        anglePIDController.setP(Constants.Swerve.angleKP);
        anglePIDController.setI(Constants.Swerve.angleKI);
        anglePIDController.setD(Constants.Swerve.angleKD);
        anglePIDController.setFF(Constants.Swerve.angleKF);
        Timer.delay(2.0);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        mDriveMotor.restoreFactoryDefaults();
        mDriveMotor.enableVoltageCompensation(12);
        mDriveMotor.setSmartCurrentLimit(Constants.Swerve.drivePeakCurrentLimit, Constants.Swerve.driveContinuousCurrentLimit);
        mDriveMotor.setOpenLoopRampRate(Constants.Swerve.openLoopRamp);
        mDriveMotor.setClosedLoopRampRate(Constants.Swerve.closedLoopRamp);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setIdleMode(Constants.Swerve.driveIdleMode);
        mDriveMotor.getEncoder().setPosition(0);

        drivePIDController = mDriveMotor.getPIDController();
        drivePIDController.setSmartMotionAllowedClosedLoopError(Constants.Swerve.driveAllowableError, 0);
        drivePIDController.setP(modConstants.drivekP);
        drivePIDController.setI(modConstants.drivekI);
        drivePIDController.setD(modConstants.drivekD);
        drivePIDController.setFF(modConstants.drivekF);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.neoToMPS(mDriveMotor.getEncoder().getVelocity(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.neoToMeters(mDriveMotor.getEncoder().getPosition(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        );
    }
}