package frc.robot.subsystems.test;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class TesterFalcon implements TesterMotor {
    private TalonFX talon;
    private String name;
    public TesterFalcon(String name, TalonFX talon) {
        this.talon = talon;
        this.name = name;
    }
    public String getName() {
        return this.name;
    }
    public void run(double velocity) {
        talon.set(ControlMode.PercentOutput, velocity);
    }
}
