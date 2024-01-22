package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake extends SubsystemBase {
  public CANSparkMax mIntake;
  public CANSparkMax secondIntake;

  public Intake() {
    mIntake = new CANSparkMax(Constants.Intake.canID, MotorType.kBrushless);
    secondIntake = new CANSparkMax(17, MotorType.kBrushless);
    configIntakeMotors();
  }

  @Override
  public void periodic() {
  }

  private void configIntakeMotors(){
    mIntake.restoreFactoryDefaults();
    mIntake.enableVoltageCompensation(12);
    mIntake.setIdleMode(Constants.Wrist.idleMode);
    secondIntake.restoreFactoryDefaults();
    secondIntake.enableVoltageCompensation(12);
    secondIntake.setIdleMode(Constants.Wrist.idleMode);
  }

  public void runIntake(double power) {
    mIntake.set(power);
    secondIntake.set(power);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
