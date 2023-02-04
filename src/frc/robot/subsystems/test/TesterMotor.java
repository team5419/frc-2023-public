package frc.robot.subsystems.test;

import frc.robot.classes.PID;

public interface TesterMotor {
    public String getName();
    public void run(double percent); 
    public double getVelocity();
    public void configurePID(PID pid);
    public void setVelocity(double velocity);
    public double getMaxVelocity();
}