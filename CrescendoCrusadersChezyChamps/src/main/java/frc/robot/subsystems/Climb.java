package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
    private CANSparkMax climbL;
    private CANSparkMax climbR;

    private SparkPIDController climbControllerL;
    private SparkPIDController climbControllerR;
    

    public Climb() {
        climbL = new CANSparkMax(Constants.Climber.motorID1, MotorType.kBrushless);
        climbL.setInverted(false);
        climbL.setIdleMode(IdleMode.kBrake);
        climbL.enableVoltageCompensation(12);
        climbL.setSmartCurrentLimit(40);
        climbR = new CANSparkMax(Constants.Climber.motorID2, MotorType.kBrushless);
        climbR.setInverted(false);
        climbR.setIdleMode(IdleMode.kBrake);
        climbR.enableVoltageCompensation(12);
        climbR.setSmartCurrentLimit(40);

        climbControllerL = climbL.getPIDController();
        climbControllerL.setFF(.0001);
        climbControllerL.setSmartMotionAllowedClosedLoopError(.8, 0);

        climbControllerR = climbR.getPIDController();
        climbControllerR.setFF(.0001);
        climbControllerR.setSmartMotionAllowedClosedLoopError(.8, 0);
        
        climbControllerL.setSmartMotionMaxVelocity(10000, 0);
        climbControllerR.setSmartMotionMaxVelocity(10000, 0);
        climbControllerL.setSmartMotionMaxAccel(90000, 0);
        climbControllerR.setSmartMotionMaxAccel(90000, 0);
    }
    
    public void setClimbHeight(Constants.Climber.Modes mode) {
        switch (mode) {
            case Down:
            climbControllerL.setReference(Constants.Climber.ClimbSetpoints.downPos, ControlType.kSmartMotion);
            climbControllerR.setReference(Constants.Climber.ClimbSetpoints.downPosR, ControlType.kSmartMotion);
                break;
        
            case Climbing:
            climbControllerL.setReference(Constants.Climber.ClimbSetpoints.climbingPos, ControlType.kSmartMotion);
            climbControllerR.setReference(Constants.Climber.ClimbSetpoints.climbingPosR, ControlType.kSmartMotion);
                break;
            case Mid:
            climbControllerL.setReference(Constants.Climber.ClimbSetpoints.midPos, ControlType.kSmartMotion);
            climbControllerR.setReference(Constants.Climber.ClimbSetpoints.midPosR, ControlType.kSmartMotion);
                break;
            case Zero:
            climbControllerL.setReference(Constants.Climber.ClimbSetpoints.zero, ControlType.kSmartMotion);
            climbControllerR.setReference(Constants.Climber.ClimbSetpoints.zeroR, ControlType.kSmartMotion);
                break;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("lPosClimb", climbL.getEncoder().getPosition());
        SmartDashboard.putNumber("rPosClimb", climbR.getEncoder().getPosition());
    }
}
