package frc.robot.auto;

import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Translation2d;

public class Bridge_Balance extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Swerve m_subsystem;
  private int balance_state;

  public Bridge_Balance(Swerve subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    balance_state = 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double error = m_subsystem.gyro.getPitch();





    if(balance_state == 1) {
      System.out.println("Hi");
      if(m_subsystem.gyro.getPitch() > 12) {
        balance_state = 2;
      }
      m_subsystem.drive(
            new Translation2d(-0.2, 0).times(Constants.Swerve.maxSpeed), 
            0 * Constants.Swerve.maxAngularVelocity, 
            false, 
            false
      );
    }
    else if(balance_state == 2) {
      System.out.println("hey");
      double pitch_error = 0 - m_subsystem.gyro.getPitch();
      double drive_power = 0;
      if(Math.abs(m_subsystem.gyro.getPitch()) > 10) {
        drive_power = pitch_error * 0.01;
      } else {
        drive_power = pitch_error * 0.00001;
      }
      m_subsystem.drive(
            new Translation2d(drive_power, 0).times(Constants.Swerve.maxSpeed), 
            0 * Constants.Swerve.maxAngularVelocity, 
            false, 
            false
      );
      // if(Math.abs(pitch_error) < 2.0) {
      //   m_subsystem.tareGyro(180.0);
      //  balance_state = 3;
      // }
    //}
    //else if(balance_state == 3){
      //Set modules to X and tare swerve
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.drive(
          new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
          0 * Constants.Swerve.maxAngularVelocity, 
          false, 
          true
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return balance_state == 3;
    return false;
  }
}
