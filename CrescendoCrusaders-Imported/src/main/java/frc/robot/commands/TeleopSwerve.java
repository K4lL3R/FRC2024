package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private CommandJoystick controller;
    private Boolean robotCentricSup;

    public TeleopSwerve(Swerve s_Swerve, CommandJoystick controller, Boolean robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        double translationVal = -MathUtil.applyDeadband(controller.getY(), Constants.stickDeadband);
        double strafeVal = -MathUtil.applyDeadband(controller.getX(), Constants.stickDeadband);
        double rotationVal = -MathUtil.applyDeadband(controller.getZ(), Constants.stickDeadband);

        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup, 
            true
        );
    }
}