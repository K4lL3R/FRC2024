package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.CANSparkMax.ControlType;

public class Wrist extends SubsystemBase {
  public CANSparkMax mWrist;
  public SparkMaxPIDController wristPIDController;
  public DutyCycleEncoder wristEnc;
  public static double wristBootDeg;
  public double wristDeg;

  public Wrist() {
    mWrist = new CANSparkMax(Constants.Wrist.canID, MotorType.kBrushless);
    configWristMotor();
    wristEnc = new DutyCycleEncoder(Constants.Wrist.encoderPort);
    Timer.delay(2.0);

    wristBootDeg = -(((wristEnc.getAbsolutePosition() - Constants.Wrist.hardStopDegOffset)*360.0)) + 207.0;
    mWrist.getEncoder().setPosition(wristConvDegtoRot(wristBootDeg));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    wristDeg = wristConvRottoDeg(mWrist.getEncoder().getPosition());
    SmartDashboard.putNumber("Wrist Deg: ", wristDeg);
    SmartDashboard.putNumber("Wrist Raw: ", wristEnc.getAbsolutePosition());
  }

  public void driveWristToDeg(double target_deg) {
    wristPIDController.setReference(wristConvDegtoRot(target_deg), ControlType.kSmartMotion);
  }

  public double wristConvDegtoRot(double deg) {
    return ((deg/360.0) * 78.125);
  }

  public double wristConvRottoDeg(double rot) {
    return ((rot / (78.125)) * 360.0);
  }

  private void configWristMotor(){
    mWrist.restoreFactoryDefaults();
    mWrist.enableVoltageCompensation(12);
    mWrist.setIdleMode(Constants.Wrist.idleMode);
    mWrist.setInverted(true);

    wristPIDController = mWrist.getPIDController();
    wristPIDController.setP(Constants.Wrist.kP);
    wristPIDController.setI(Constants.Wrist.kI);
    wristPIDController.setD(Constants.Wrist.kD);
    wristPIDController.setFF(Constants.Wrist.kF);
    wristPIDController.setSmartMotionAllowedClosedLoopError(0.01, 0);
    wristPIDController.setSmartMotionMaxVelocity(Constants.Wrist.maxVel, 0);
    wristPIDController.setSmartMotionMaxAccel(Constants.Wrist.maxAccel, 0);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
