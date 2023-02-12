package frc.robot.subsystems.test;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.classes.PID;

public class TesterNeo implements TesterMotor {
    private CANSparkMax neo;
    private RelativeEncoder encoder;
    private String name;
    private PIDController pidController;
    public TesterNeo(String name, CANSparkMax neo) {
        this.neo = neo;
        this.encoder = neo.getEncoder();
        this.name = name;
        this.pidController = new PIDController(0.0, 0.0, 0.0);
    }
    public final double getMaxVelocity() {
        return 5700.0;
    }
    public void configurePID(PID pid) {
        pidController.setP(pid.p);
        pidController.setI(pid.i);
        pidController.setD(pid.d);
    }
    public String getName() {
        return this.name;
    }
    public void run(double velocity) {
        neo.set(velocity);
    }
    public double getVelocity() {
        return encoder.getVelocity();
    }
    public double getPosition() {
        return encoder.getPosition();
    }
    public void setVelocity(double velocity) {
        neo.set(pidController.calculate(getVelocity(), velocity));
    }
}
