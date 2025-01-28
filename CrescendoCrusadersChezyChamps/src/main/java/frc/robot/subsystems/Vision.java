package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

public class Vision extends SubsystemBase {
    
    public Vision() {
        LimelightHelpers.setCameraPose_RobotSpace("limelight-upfront", -0.2667, -0.2921, 0.64135, 0, 15, 180);
        // LimelightHelpers.setCameraPose_RobotSpace("limelight-dfront", 0.2286, 0, 0.6604, 0, -37, 0);
        // LimelightHelpers.setCameraPose_RobotSpace("limelight3", 0, 0, 0, 0, 0, 0);
        
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ll 1 distance", getDistance());
        double id = LimelightHelpers.getFiducialID("limelight-dfront");
        SmartDashboard.putNumber("ApriltagID", id);
        SmartDashboard.putNumber("tangent", Math.tan(47 - LimelightHelpers.getTY("limelight-dfront")));
        SmartDashboard.putNumber("distance2", getDistance2());
    }

    public static double getDistance() {
        return 34 / Math.tan((47 - LimelightHelpers.getTY("limelight-dfront")) * (Math.PI / 180));
    }

    public static double getDistance2() {
        return 27.5 / Math.tan((15 + LimelightHelpers.getTY("limelight-upfront")) * (Math.PI / 180));
    }

    public static double limelight_aim_proportional() {    
        // kP (constant of proportionality)
        // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
        // if it is too high, the robot will oscillate.
        // if it is too low, the robot will never reach its target
        // if the robot never turns in the correct direction, kP should be inverted.
            double kP = .0078;
    
            // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
            // your limelight 3 feed, tx should return roughly 31 degrees.
            double targetingAngularVelocity = LimelightHelpers.getTX("limelight-upfront") * kP;
    
            // convert to radians per second for our drive method
            targetingAngularVelocity *= RobotContainer.MaxAngularRate;
    
            //invert since tx is positive when the target is to the right of the crosshair
            targetingAngularVelocity *= -1.0;
    
            return targetingAngularVelocity;
        }


}
