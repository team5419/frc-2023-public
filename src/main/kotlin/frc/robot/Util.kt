package frc.robot;
import frc.robot.Constants.Drive;
object Util {
    public fun nativeUnitsToMetersPerSecond(units: Double): Double {
        return (units / Drive.ticksPerRotation) * Drive.wheelCircumference * 10.0
    }
    public fun metersPerSecondToNativeUnits(units: Double): Double {
        return ((units / Drive.wheelCircumference) * Drive.ticksPerRotation) / 10.0
    }
}