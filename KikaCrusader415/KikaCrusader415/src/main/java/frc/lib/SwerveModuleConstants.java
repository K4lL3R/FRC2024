package frc.lib;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int steerMotorID;
    public final int canCoderID;
    public final Rotation2d angleOffset;
    public final double drivekP;
    public final double drivekI;
    public final double drivekD;
    public final double drivekF;

    public SwerveModuleConstants(int driveMotorID, int steerMotorID, int canCoderID, Rotation2d angleOffset, double drivekP, double drivekI, double drivekD, double drivekF) {
        this.driveMotorID = driveMotorID;
        this.steerMotorID = steerMotorID;
        this.canCoderID = canCoderID;
        this.angleOffset = angleOffset;
        this.drivekP = drivekP;
        this.drivekI = drivekI;
        this.drivekD = drivekD;
        this.drivekF = drivekF;
    }
}