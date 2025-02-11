package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
    public CANSparkMax climbMotorL;
    public CANSparkMax climbMotorR;
    public SparkPIDController climbControllerL;
    public SparkPIDController climbControllerR;

    public Climb() {
        climbMotorL = new CANSparkMax(31, MotorType.kBrushless);
        climbMotorL.setInverted(false);
        climbMotorL.setIdleMode(IdleMode.kBrake);
        climbMotorL.enableVoltageCompensation(12);
        climbMotorL.setSmartCurrentLimit(40);
        climbMotorR = new CANSparkMax(32, MotorType.kBrushless);
        climbMotorR.setInverted(false);
        climbMotorR.setIdleMode(IdleMode.kBrake);
        climbMotorR.enableVoltageCompensation(12);
        climbMotorR.setSmartCurrentLimit(40);

        climbControllerL = climbMotorL.getPIDController();
        climbControllerL.setFF(.0001);
        climbControllerL.setSmartMotionAllowedClosedLoopError(.8, 0);

        climbControllerR = climbMotorR.getPIDController();
        climbControllerR.setFF(.0001);
        climbControllerR.setSmartMotionAllowedClosedLoopError(.8, 0);
        
        climbControllerL.setSmartMotionMaxVelocity(Constants.Climb.maxVel, 0);
        climbControllerR.setSmartMotionMaxVelocity(Constants.Climb.maxVel, 0);
        climbControllerL.setSmartMotionMaxAccel(Constants.Climb.maxAccel, 0);
        climbControllerR.setSmartMotionMaxAccel(Constants.Climb.maxAccel, 0);

        
    }

    public void moveClimb(Constants.Climb.Position pos) {
        switch(pos) {
            case Up:    climbControllerL.setSmartMotionMaxVelocity(Constants.Climb.maxVel, 0);
                        climbControllerR.setSmartMotionMaxVelocity(Constants.Climb.maxVel, 0);
                        climbControllerL.setReference(Constants.Climb.GlobalSetpoints.elevatorUpPos, ControlType.kSmartMotion);
                        climbControllerR.setReference(-Constants.Climb.GlobalSetpoints.elevatorUpPos, ControlType.kSmartMotion);
            break;
            case Down:  climbControllerL.setSmartMotionMaxVelocity(Constants.Climb.maxVel, 0);
                        climbControllerR.setSmartMotionMaxVelocity(Constants.Climb.maxVel, 0);
                        climbControllerL.setReference(Constants.Climb.GlobalSetpoints.elevatorDefaultPos, ControlType.kSmartMotion);
                        climbControllerR.setReference(-Constants.Climb.GlobalSetpoints.elevatorDefaultPos, ControlType.kSmartMotion);
            break;
            case Mid:   climbControllerL.setSmartMotionMaxVelocity(Constants.Climb.maxVel, 0);
                        climbControllerR.setSmartMotionMaxVelocity(Constants.Climb.maxVel, 0);
                        climbControllerL.setReference(Constants.Climb.GlobalSetpoints.midPos, ControlType.kSmartMotion);
                        climbControllerR.setReference(-Constants.Climb.GlobalSetpoints.midPos, ControlType.kSmartMotion);
            break;
            case Amp:   climbControllerL.setSmartMotionMaxVelocity(Constants.Climb.maxVel, 0);
                        climbControllerR.setSmartMotionMaxVelocity(Constants.Climb.maxVel, 0);
                        climbControllerL.setReference(Constants.Climb.GlobalSetpoints.ampElevator, ControlType.kSmartMotion);
                        climbControllerR.setReference(-Constants.Climb.GlobalSetpoints.ampElevator, ControlType.kSmartMotion);
            break;
        }
    }



    @Override
    public void periodic() {
    }
}
//config and run motor to raise and pull down lift for climb on chain