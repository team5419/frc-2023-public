package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface GenericShootIntake {
    public void shoot(String height);
    public void stop(String height);
    public void setup(String height, boolean firstTime);
    public double getAngle();
    public double getOffset();
    public double getDistance(String height);
    public double getLimelightDistance(String height);
    public SubsystemBase subsystem();
}