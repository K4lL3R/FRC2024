// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
// import frc.robot.subsystems.Drive;

// public class TeleopDrive extends Command {
//   private Drive drive;
//   private CommandJoystick controller;

//   public TeleopDrive(Drive drive, CommandJoystick controller) {
//     this.drive = drive;
//     addRequirements(drive);

//     this.controller = controller;
//   }

//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     drive.runDrive(controller);
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
