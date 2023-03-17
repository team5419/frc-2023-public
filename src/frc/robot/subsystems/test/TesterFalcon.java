package frc.robot.subsystems.test;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.classes.PID;

public class TesterFalcon implements TesterMotor {
    private TalonFX talon;
    private String name;
    public TesterFalcon(String name, TalonFX talon) {
        this.talon = talon;
        this.name = name;
    }
    public final double getMaxVelocity() {
        return 22000.0;
    }
    public TesterFalcon configurePID(PID pid) {
        talon.config_kP(0, pid.p);
        talon.config_kI(0, pid.i);
        talon.config_kD(0, pid.d);
        talon.config_kF(0, pid.f);
        return this;
    }
    public String getName() {
        return this.name;
    }
    public void run(double velocity) {
        //System.out.println("directly setting motor to " + velocity);
        talon.set(ControlMode.PercentOutput, velocity);
    }
    public void stop() {
        run(0.0);
    }
    public double getVelocity() {
        return talon.getSelectedSensorVelocity(0);
    }
    public void setVelocity(double velocity) {
        talon.set(ControlMode.Velocity, velocity);
    }
    public double getPosition() {
        return talon.getSelectedSensorPosition(0);
    }
}
