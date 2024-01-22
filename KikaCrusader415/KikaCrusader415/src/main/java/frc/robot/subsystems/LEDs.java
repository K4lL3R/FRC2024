package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LEDs extends SubsystemBase {
  public Spark mLEDs;

  public LEDs() {
    mLEDs = new Spark(0);
  }

  @Override
  public void periodic() {
  }

  public void SetLED(int mode){
    if(mode == 1) {
      //Purple
      mLEDs.set(0.73);  
    }
    else if(mode == 2) {
      //Yellow
      mLEDs.set(0.57);  
    }
    else {
      //Red
      mLEDs.set(-0.11);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
