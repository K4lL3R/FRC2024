package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class SwerveDrive extends Command {
    private CommandSwerveDrivetrain s_Swerve;
    private Constants.TunerConstants.DriveModes mode;
    private CommandJoystick controller;

      //fieldcentric
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(RobotContainer.MaxSpeed * 0.05).withRotationalDeadband(RobotContainer.MaxAngularRate * 0.03) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  //robotcentric for tracking
    public static final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
      .withDeadband(RobotContainer.MaxSpeed * 0.05).withRotationalDeadband(RobotContainer.MaxAngularRate * 0.03) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public SwerveDrive(CommandSwerveDrivetrain swerve, Constants.TunerConstants.DriveModes mode, CommandJoystick controller) {
        s_Swerve = swerve;
        this.mode = mode;
        this.controller = controller;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        switch (mode) {
            case FieldCentric:
                s_Swerve.applyRequest(() -> drive.withVelocityX(-MathUtil.applyDeadband(controller.getY(), 0.1) * RobotContainer.MaxSpeed) // Drive forward with
                // negative Y (forward)
                  .withVelocityY(-MathUtil.applyDeadband(controller.getX(), 0.1) * RobotContainer.MaxSpeed) // Drive left with negative X (left)
                  .withRotationalRate(-MathUtil.applyDeadband(controller.getZ(), 0.1) * RobotContainer.MaxAngularRate)); // Drive counterclockwise with negative X (left))
                break;
        
            case RobotCentric:
                s_Swerve.applyRequest(() -> driveRobotCentric.withVelocityX(-MathUtil.applyDeadband(controller.getY(), 0.1) * RobotContainer.MaxSpeed) // Drive forward with
                // negative Y (forward)
                  .withVelocityY(-MathUtil.applyDeadband(controller.getX(), 0.1) * RobotContainer.MaxSpeed) // Drive left with negative X (left)
                  .withRotationalRate(-MathUtil.applyDeadband(controller.getZ(), 0.1) * RobotContainer.MaxAngularRate)); // Drive counterclockwise with negative X (left))
                break;
        }
    }
}
