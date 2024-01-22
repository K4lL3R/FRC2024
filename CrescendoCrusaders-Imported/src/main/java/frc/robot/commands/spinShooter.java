// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.*;;

// public class spinShooter extends Command {
//   Shooter shooter;
//   double power;
  
//   public spinShooter(Shooter shooter, double power) {
//     this.shooter = shooter;
//     addRequirements(shooter);

//     this.power = power;
//   }

//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     shooter.runShooter(power);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
