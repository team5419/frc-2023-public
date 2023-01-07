package frc.robot;
import frc.robot.Constants.Drive;
public class Util {
    public static double nativeUnitsToMetersPerSecond(double units) {
        return (units / Drive.ticksPerRotation) * Drive.wheelCircumference * 10.0;
    }
    public static double metersPerSecondToNativeUnits(double units) {
        return ((units / Drive.wheelCircumference) * Drive.ticksPerRotation) / 10.0;
    }
}