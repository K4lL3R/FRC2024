package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

public class Elevator extends SubsystemBase {
  public CANSparkMax mElevator;
  public SparkMaxPIDController elevatorPIDController;
  public DutyCycleEncoder elevatorEnc;
  public static double elevatorBootHeight;
  public double elevatorHeight;

  public Elevator() {
    mElevator = new CANSparkMax(Constants.Elevator.canID, MotorType.kBrushless);
    configElevatorMotor();
    elevatorEnc = new DutyCycleEncoder(Constants.Elevator.encoderPort);
    Timer.delay(2.0);

    elevatorBootHeight = (((Constants.Elevator.mappingOffset - elevatorEnc.getAbsolutePosition())*360.0)*(5.520 / 360.0)) + Constants.Elevator.mappingHeight;
    mElevator.getEncoder().setPosition(elevatorConvHeighttoRot(elevatorBootHeight));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Raw: ", elevatorEnc.getAbsolutePosition());
    elevatorHeight = elevatorConvRottoHeight(mElevator.getEncoder().getPosition());
    SmartDashboard.putNumber("Elevator Height: ", elevatorHeight);
  }

  public void driveElevatorToHeight(double height) {
    elevatorPIDController.setReference(elevatorConvHeighttoRot(height), ControlType.kSmartMotion);
  }
  
  public double elevatorConvHeighttoRot(double height) {
    return (((height - Constants.Elevator.mappingHeight) / 5.520) * 9.7);
  }

  public double elevatorConvRottoHeight(double rot) {
    return (((rot / 9.7) * 5.520) + Constants.Elevator.mappingHeight);
  }

  private void configElevatorMotor(){
    mElevator.restoreFactoryDefaults();
    mElevator.enableVoltageCompensation(12);
    mElevator.setIdleMode(Constants.Elevator.idleMode);
    mElevator.setInverted(true);

    elevatorPIDController = mElevator.getPIDController();
    elevatorPIDController.setP(0);
    elevatorPIDController.setI(Constants.Elevator.kI);
    elevatorPIDController.setD(0);
    elevatorPIDController.setFF(Constants.Elevator.kF);
    elevatorPIDController.setSmartMotionAllowedClosedLoopError(0.01, 0);
    elevatorPIDController.setSmartMotionMaxVelocity(Constants.Elevator.maxVel, 0);
    elevatorPIDController.setSmartMotionMaxAccel(Constants.Elevator.maxAccel, 0);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
