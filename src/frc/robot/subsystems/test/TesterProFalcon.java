package frc.robot.subsystems.test;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenixpro.hardware.TalonFX;

import frc.robot.classes.PID;

public class TesterProFalcon implements TesterMotor {
    private TalonFX talon;
    private TalonFXConfiguration config;
    private String name;
    public TesterProFalcon(String name, TalonFX talon, TalonFXConfiguration config) {
        this.talon = talon;
        this.name = name;
        this.config = config;
        talon.getConfigurator().apply(config);
        //System.out.println(config.MotorOutput.NeutralMode == NeutralModeValue.Brake);
    }
    public final double getMaxVelocity() {
        return 22000.0;
    }
    public TesterProFalcon configurePID(PID pid) {
        config.Slot0.kP = pid.p;
        config.Slot0.kI = pid.i;
        config.Slot0.kD = pid.d;
        talon.getConfigurator().refresh(config);
        return this;
    }
    public void stop() {
        run(0.0);
    }
    public String getName() {
        return this.name;
    }
    public void run(double velocity) {
        //System.out.println("directly setting motor to " + velocity);
        talon.set(velocity);
    }
    public double getVelocity() {
        return talon.getVelocity().getValue();
    }
    public void setVelocity(double velocity) {
        System.out.println("setting velocity to " + velocity);
        talon.setControl(new VelocityTorqueCurrentFOC(velocity));
    }
    public double getPosition() {
        return talon.getPosition().getValue();
    }
}
