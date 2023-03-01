package frc.robot.subsystems.test;

import frc.robot.classes.PID;

public interface TesterMotor {
    public String getName();
    public void run(double percent); 
    public double getVelocity();
    public TesterMotor configurePID(PID pid);
    public void setVelocity(double velocity);
    public double getMaxVelocity();
    public double getPosition();
    public void stop();
}