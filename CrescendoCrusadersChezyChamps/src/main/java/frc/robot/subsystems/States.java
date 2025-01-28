package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.AngleChanger.Angles;
import frc.robot.Constants.TunerConstants.DriveModes;
import frc.robot.commands.SingleSubsystemCommands.SetWristPosTracking;
import frc.robot.commands.StateCommands.AfterAmpState;
import frc.robot.commands.StateCommands.AmpingState;
import frc.robot.commands.StateCommands.ClimbingState;
import frc.robot.commands.StateCommands.DefaultState;
import frc.robot.commands.StateCommands.IntakingState;
import frc.robot.commands.StateCommands.ManualShooting;
import frc.robot.commands.StateCommands.PassingState;
import frc.robot.commands.StateCommands.PrepShotState;
import frc.robot.commands.StateCommands.ShootingState;
import frc.robot.commands.StateCommands.TrappingState;

public class States {

    public static enum RobotState {
        Shooting,
        Amping,
        Climbing,
        Trapping,
        Intaking,
        PrepShotCloseTrus,
        PrepShotWoofer,
        PrepShotBackward,
        PrepShotTracking,
        Passing,
        Default,
        AfterAmp,
        ManualWoofer,
        BackwardWoofer
    }

   
    public static Command run(States.RobotState state) {
        switch (state) {
            case Shooting:
                return new ShootingState();
                   
            case Amping:
                return new AmpingState();

            case Climbing:
                return new ClimbingState();

            case Trapping:
                return new TrappingState();
            case Intaking:
                return new IntakingState();
            case PrepShotCloseTrus:
                return new PrepShotState(Angles.CloseTruss);
            case PrepShotWoofer:
                return new PrepShotState(Angles.Woofer);
            case PrepShotBackward:
                return new PrepShotState(Angles.BackwardWoofer);

            case PrepShotTracking:
                return new SetWristPosTracking(RobotContainer.s_ShooterWrist).alongWith(RobotContainer.s_Swerve.setSwerveState(DriveModes.RobotCentric, RobotContainer.controller));
            case AfterAmp:
                return new AfterAmpState();
            case Passing:
                return new PassingState();
            case ManualWoofer:
                return new ManualShooting(Angles.Woofer);
            case BackwardWoofer:
                return new ManualShooting(Angles.BackwardWoofer);
            default:
                return new DefaultState();
            
        }
    }

}
