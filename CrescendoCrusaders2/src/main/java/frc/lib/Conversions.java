package frc.lib;

public class Conversions {
    public static double neoToDegrees(double motorRotations, double gearRatio) {
        return motorRotations * (360.0 / gearRatio);
    }

    public static double degreesToNeo(double degrees, double gearRatio) {
        return degrees / (360.0 / gearRatio);
    }

    public static double neoToMPS(double motorRPM, double circumference, double gearRatio){
        return ((motorRPM / gearRatio) * circumference) / 60.0;
    }
    
    public static double MPSToNeo(double velocity, double circumference, double gearRatio){
        return ((velocity * 60.0) / circumference) * gearRatio;
    }

    public static double neoToMeters(double motorRotations, double circumference, double gearRatio){
        return motorRotations * (circumference / gearRatio);
    }

    public static double talonToDeg(double motorRot, double gearRatio) {
        return motorRot * (360 / (gearRatio * 2048));
    }

    public static double degToTalon(double deg, double gearRatio) {
        return deg / (360 / (gearRatio * 2048));
    }

    public static double talonToMPS(double velocitycounts, double circumference, double gearRatio){
        double wheelRPM = talonToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    public static double talonToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / 2048.0);        
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    public static double RPMToTalon(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        double sensorCounts = motorRPM * (2048.0 / 600.0);
        return sensorCounts;
    }

    public static double MPSToTalon(double velocity, double circumference, double gearRatio){
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMToTalon(wheelRPM, gearRatio);
        return wheelVelocity;
    }

    public static double talonToMeters(double positionCounts, double circumference, double gearRatio){
        return positionCounts * (circumference / (gearRatio * 2048.0));
    }
}