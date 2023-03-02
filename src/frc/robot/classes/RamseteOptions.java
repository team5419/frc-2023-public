package frc.robot.classes;

public class RamseteOptions {
    public boolean teamRelative;
    public boolean preventDrive;
    public boolean speedLimit;
    public double epsilonMultiplier;
    public int turnToTag;
    public double maxSpeed;
    public double time;
    public RamseteOptions(double maxSpeed) {
        this.teamRelative = true;
        this.preventDrive = false;
        this.speedLimit = true;
        this.epsilonMultiplier = 1.0;
        this.turnToTag = -1;
        this.maxSpeed = maxSpeed;
        time = 0.0;
    }
    public RamseteOptions() {
        this.teamRelative = true;
        this.preventDrive = false;
        this.speedLimit = true;
        this.epsilonMultiplier = 1.0;
        this.turnToTag = -1;
        this.maxSpeed = -1.0;
        time = 0.0;
    }
    public RamseteOptions(boolean teamRelative, boolean preventDrive) {
        this.teamRelative = teamRelative;
        this.preventDrive = preventDrive;
        this.speedLimit = true;
        this.epsilonMultiplier = 1.0;
        this.turnToTag = -1;
        this.maxSpeed = -1.0;
        time = 0.0;
    }
    public RamseteOptions(boolean speedLimit, double epsilonMultiplier) {
        this.speedLimit = speedLimit;
        this.epsilonMultiplier = epsilonMultiplier;
        this.teamRelative = true;
        this.preventDrive = false;
        this.turnToTag = -1;
        this.maxSpeed = -1.0;
        time = 0.0;
    }
    public RamseteOptions(boolean teamRelative, boolean preventDrive, boolean speedLimit, double epsilonMultiplier, int turnToTag, double maxSpeed, double time) {
        this.teamRelative = teamRelative;
        this.preventDrive = preventDrive;
        this.speedLimit = speedLimit;
        this.epsilonMultiplier = epsilonMultiplier;
        this.turnToTag = turnToTag;
        this.maxSpeed = -1.0;
        this.time = time;
    }
    public RamseteOptions(int turnToTag, double time) {
        this.teamRelative = true;
        this.preventDrive = false;
        this.speedLimit = true;
        this.epsilonMultiplier = 1.0;
        this.turnToTag = turnToTag;
        this.maxSpeed = -1.0;
        this.time = time;
    }
}
