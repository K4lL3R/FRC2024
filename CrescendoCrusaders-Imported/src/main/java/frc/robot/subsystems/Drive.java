// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

// import com.revrobotics.CANSparkMax;


// public class Drive extends SubsystemBase{
//   CANSparkMax lDrive1;
//   CANSparkMax lDrive2;
//   CANSparkMax rDrive1;
//   CANSparkMax rDrive2;
  
//   public Drive() {
//      lDrive1= new CANSparkMax(0, MotorType.kBrushless);
//      lDrive2= new CANSparkMax(1, MotorType.kBrushless);
//      rDrive1= new CANSparkMax(2, MotorType.kBrushless);
//      rDrive2= new CANSparkMax(3, MotorType.kBrushless);
//   }

//   public void runDrive(CommandJoystick controller) {
//     lDrive1.set(-controller.getY());
//     lDrive2.set(-controller.getY());
//     rDrive1.set(controller.getZ());
//     rDrive2.set(controller.getZ());
//   }
// }
