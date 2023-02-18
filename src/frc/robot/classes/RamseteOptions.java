package frc.robot.classes;

public class RamseteOptions {
    public boolean teamRelative;
    public boolean preventDrive;
    public boolean speedLimit;
    public double epsilonMultiplier;
    public int turnToTag;
    public RamseteOptions() {
        this.teamRelative = true;
        this.preventDrive = false;
        this.speedLimit = true;
        this.epsilonMultiplier = 1.0;
        this.turnToTag = -1;
    }
    public RamseteOptions(boolean teamRelative, boolean preventDrive) {
        this.teamRelative = teamRelative;
        this.preventDrive = preventDrive;
        this.speedLimit = true;
        this.epsilonMultiplier = 1.0;
        this.turnToTag = -1;
    }
    public RamseteOptions(boolean speedLimit, double epsilonMultiplier) {
        this.speedLimit = speedLimit;
        this.epsilonMultiplier = epsilonMultiplier;
        this.teamRelative = true;
        this.preventDrive = false;
        this.turnToTag = -1;
    }
    public RamseteOptions(boolean teamRelative, boolean preventDrive, boolean speedLimit, double epsilonMultiplier, int turnToTag) {
        this.teamRelative = teamRelative;
        this.preventDrive = preventDrive;
        this.speedLimit = speedLimit;
        this.epsilonMultiplier = epsilonMultiplier;
        this.turnToTag = turnToTag;
    }
    public RamseteOptions(int turnToTag) {
        this.teamRelative = true;
        this.preventDrive = false;
        this.speedLimit = true;
        this.epsilonMultiplier = 1.0;
        this.turnToTag = turnToTag;
    }
}
