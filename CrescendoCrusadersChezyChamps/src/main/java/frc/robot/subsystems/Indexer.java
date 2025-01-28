package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
    private CANSparkMax indexer1;
    private CANSparkMax indexer2;
    
    public Indexer() {
        indexer1 = new CANSparkMax(Constants.indexer1ID, MotorType.kBrushless);
        indexer1.enableVoltageCompensation(12);
        indexer1.setInverted(false);
        indexer1.setIdleMode(IdleMode.kBrake);

        indexer2 = new CANSparkMax(Constants.indexer2ID, MotorType.kBrushless);
        indexer2.enableVoltageCompensation(12);
        indexer2.setInverted(true);
        indexer2.setIdleMode(IdleMode.kBrake);
    }

    public void setIndexerSpeed(double speed) {
        indexer1.set(speed);
        indexer2.set(speed);
    }

    @Override
    public void periodic() {
        
    }
}
