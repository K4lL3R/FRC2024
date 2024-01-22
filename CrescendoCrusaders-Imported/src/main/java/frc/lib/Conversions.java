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
}