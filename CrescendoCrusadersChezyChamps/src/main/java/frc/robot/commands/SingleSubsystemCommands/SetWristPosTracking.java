package frc.robot.commands.SingleSubsystemCommands;

import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterWrist;
import frc.robot.subsystems.Vision;

public class SetWristPosTracking extends Command {
    private ShooterWrist wrist;
    private Timer timer;
    private boolean isFinished = false;
    
    public SetWristPosTracking(ShooterWrist s_Wrist) {
        wrist = s_Wrist;

        addRequirements(s_Wrist);
    }

    @Override
    public void initialize() {
        // timer.start();
    }

    @Override
    public void execute() {
        // if (wrist.distanceFar <= Constants.AngleChanger.distancearrayFar[7]) {
        //     wrist.interpolatedAngle(wrist.distance);
        // } else {
        //     wrist.interpolatedAngleFar(wrist.distanceFar);
        // }
        wrist.setWristTrackPos(Vision.getDistance2());
        // if (LimelightHelpers.getFiducialID("limelight-upfront") == -1) {
        //     timer.reset();
        //     isFinished = false;
        // }

        // if (timer.hasElapsed(0.1)) {
        //     isFinished = true;
        // }
        // SmartDashboard.putBoolean("isWristFinised", isFinished);
    }

    @Override
    public boolean isFinished() {
        // if (LimelightHelpers.getFiducialID("limelight-upfront") != -1) {
        //     Timer.delay(0.2);
        //     if (Timer.getFPGATimestamp() < currentTime + 0.1){
        //         if (LimelightHelpers.getFiducialID("limelight-upfront") == -1) {
        //             // desiredPos = Constants.AngleChanger.ANGLESINTERPOLATE_DOUBLE_TREE_MAP.get(distanceFar);
        //             // shooterWrist.setControl(new MotionMagicExpoVoltage(desiredPos));
        //             return true;
        //         }
        //     }            
        // }
        if (wrist.inPosition()) {
            return true;
        }
        return false;
        // return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        // wrist.desiredPos = wrist.desiredPos;
        // wrist.wristSetControl(wrist.getShooterPos());
        wrist.setWristSpeed(-0.025);
    }
}
