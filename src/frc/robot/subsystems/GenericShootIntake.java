package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface GenericShootIntake {
    public void shoot(String height);
    public void stop();
    public void setup(String height);
    public SubsystemBase subsystem();
}