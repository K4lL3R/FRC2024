package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
    public CANSparkMax climbMotorL;
    public CANSparkMax climbMotorR;
    public SparkPIDController climbControllerL;
    public SparkPIDController climbControllerR;
    public DutyCycleEncoder elevatorEncL;
    public DutyCycleEncoder elevatorEncR;

    public Climb() {
        climbMotorL = new CANSparkMax(31, MotorType.kBrushless);
        climbMotorL.enableVoltageCompensation(12);
        climbMotorL.setIdleMode(IdleMode.kBrake);
        climbMotorL.setInverted(true);

        climbMotorR = new CANSparkMax(32, MotorType.kBrushless);
        climbMotorR.enableVoltageCompensation(12);
        climbMotorR.setIdleMode(IdleMode.kBrake);
        climbMotorR.setInverted(true);

        climbControllerL = climbMotorL.getPIDController();
        climbControllerL.setFF(0.0005);
        climbControllerL.setSmartMotionMaxVelocity(Constants.Climb.maxVel, 0);
        climbControllerL.setSmartMotionMaxAccel(Constants.Climb.maxAccel, 0);

        climbControllerR = climbMotorR.getPIDController();
        climbControllerR.setFF(0.0005);
        climbControllerR.setSmartMotionMaxVelocity(Constants.Climb.maxVel, 0);
        climbControllerR.setSmartMotionMaxAccel(Constants.Climb.maxAccel, 0);
    }

    public void moveClimb(Constants.Climb.Position pos) {
        switch(pos) {
            case Up:    climbControllerL.setReference(Constants.Climb.GlobalSetpoints.elevatorUpPos / 37, ControlType.kSmartMotion);
                        climbControllerR.setReference(Constants.Climb.GlobalSetpoints.elevatorUpPos / 37, ControlType.kSmartMotion);
            break;
            case Down:  climbControllerL.setReference(Constants.Climb.GlobalSetpoints.elevatorDefaultPos / 37, ControlType.kSmartMotion);
                        climbControllerR.setReference(Constants.Climb.GlobalSetpoints.elevatorDefaultPos / 37, ControlType.kSmartMotion);
            break;
        }

    }
}
//config and run motor to raise and pull down lift for climb on chain