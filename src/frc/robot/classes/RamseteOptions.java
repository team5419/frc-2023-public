package frc.robot.classes;

public class RamseteOptions {
    public boolean teamRelative;
    public boolean preventDrive;
    public boolean speedLimit;
    public double epsilonMultiplier;
    public RamseteOptions() {
        this.teamRelative = true;
        this.preventDrive = false;
        this.speedLimit = true;
        this.epsilonMultiplier = 1.0;
    }
    public RamseteOptions(boolean teamRelative, boolean preventDrive) {
        this.teamRelative = teamRelative;
        this.preventDrive = preventDrive;
        this.speedLimit = true;
        this.epsilonMultiplier = 1.0;
    }
    public RamseteOptions(boolean speedLimit, double epsilonMultiplier) {
        this.speedLimit = speedLimit;
        this.epsilonMultiplier = epsilonMultiplier;
        this.teamRelative = true;
        this.preventDrive = false;
    }
}
