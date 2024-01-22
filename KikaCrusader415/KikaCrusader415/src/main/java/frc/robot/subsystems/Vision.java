// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.LimelightHelpers;

// public class Vision extends SubsystemBase {
//   NetworkTable table;
//   NetworkTableEntry tx;
//   LimelightHelpers.LimelightResults llresults;
//   public Vision () {
//     table = NetworkTableInstance.getDefault().getTable("Limelight");
//     tx = table.getEntry("tx");
//   }

//   @Override
//   public void periodic() {
//     double x = tx.getDouble(0.0);
//     SmartDashboard.putNumber("LimelightX", x);
//     llresults = LimelightHelpers.getLatestResults("");
//     SmartDashboard.putNumberArray("botPose", LimelightHelpers.getBotPose(""));
//   }
// }
