package frc.robot.subsystems.test;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class TesterSetting {
    private double motorDefault;
    private GenericEntry entry;
    private boolean velocityControl;
    private double offset;
    public TesterSetting(boolean velocityControl, double motorDefaults) {
        this.velocityControl = velocityControl;
        this.motorDefault = motorDefaults;
        this.offset = 0.0;
    }
    public TesterSetting(double motorDefaults) {
        this.velocityControl = false;
        this.motorDefault = motorDefaults;
        this.offset =0.0;
    }
    public TesterSetting(boolean velocityControl) {
        this.velocityControl = velocityControl;
        this.motorDefault = 0.0;
        this.offset =0.0;
    }
    public TesterSetting() {
        this.velocityControl = false;
        this.motorDefault = 0.0;
        this.offset = 0.0;
    }
    public void initialize(ShuffleboardTab tab, TesterMotor motor, String name, int x, int y) {
        double max = velocityControl ? motor.getMaxVelocity() : 1.0;
        entry = tab.add(name + ": " + motor.getName(), motorDefault)
            .withPosition(x * 2, y)
            .withSize(2, 1) 
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -max, "max", max))
            .getEntry();
    }
    public void setMotor(TesterMotor motor) {
        double setpoint = getSetpoint();
        if(velocityControl) {
            motor.setVelocity(setpoint);
        } else {
            motor.run(setpoint);
        }
    }
    public void changeOffset(double value, double maxVelocity) {
        if(velocityControl) {
            offset += value * maxVelocity;
        } else {
            offset += value;
        }
    }
    public double getSetpoint() {
        return (entry == null ? motorDefault : entry.getDouble(motorDefault)) + offset;
    }
}
