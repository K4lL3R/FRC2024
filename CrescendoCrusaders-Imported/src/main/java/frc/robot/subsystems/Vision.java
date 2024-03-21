package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LimelightHelpers;

public class Vision extends SubsystemBase {
    public static NetworkTable table;
    public static LimelightHelpers.LimelightResults results;
    public static double tx;

    public Vision() {

    }

    /*shooter angle for alignment = inverse tangent of 
        (height of AprilTag - height of shooter wrist vertex) 
        / 
        (getGroundDistanceFromTag() + distance from camera to shooter wrist vertex)) 
        
     * need Translation2d vector for swerve tracking + RobotCentric
    */



    @Override
    public void periodic() {
        table = LimelightHelpers.getLimelightNTTable("");
        results = LimelightHelpers.getLatestResults("");
        tx = LimelightHelpers.getLimelightNTTableEntry("", "tx").getValue().getDouble();
    }
}
