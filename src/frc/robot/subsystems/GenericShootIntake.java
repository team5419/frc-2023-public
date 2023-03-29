package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface GenericShootIntake {
    public void shoot(String height);
    public void stop(String height);
    public void setup(String height);
    public double getAngle();
    public boolean prepsByDefault();
    public boolean donePrepping(String height);
    public SubsystemBase subsystem();
}