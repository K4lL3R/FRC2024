package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class InOuttake extends SubsystemBase {
    public static CANSparkMax shooterMotor;
    public static CANSparkMax armOuttakeMotor;
    public static CANSparkMax wristIntakeMotor;
    public static DigitalInput sensor;

    public InOuttake() {
        shooterMotor = new CANSparkMax(0, MotorType.kBrushless);
        armOuttakeMotor = new CANSparkMax(42, MotorType.kBrushless);
        wristIntakeMotor = new CANSparkMax(41, MotorType.kBrushless);

        sensor = new DigitalInput(0);
        
    }

    public void RunIntakeOuttakeMotors(CANSparkMax motor, double power) {
        motor.set(power);
    }

    public void sensorStopMotor(CANSparkMax motor) {

    }
}
//config and methods to run each intake/outtake motor
//outtake from arm for trap, run shooter, intake from wrist
//may need delay for shooter to spin up (here or in the sequences files)
//sensor to detect if note has been launched?