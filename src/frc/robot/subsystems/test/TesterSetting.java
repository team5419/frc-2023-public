package frc.robot.subsystems.test;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class TesterSetting {
    private double[] motorDefaults;
    private GenericEntry[] entries;
    private boolean velocityControl;
    public TesterSetting(boolean velocityControl, double[] motorDefaults) {
        this.velocityControl = velocityControl;
        this.motorDefaults = motorDefaults;
    }
    public TesterSetting(double[] motorDefaults) {
        this.velocityControl = false;
        this.motorDefaults = motorDefaults;
    }
    public TesterSetting(boolean velocityControl) {
        this.velocityControl = velocityControl;
        this.motorDefaults = null;
    }
    public TesterSetting() {
        this.velocityControl = false;
        this.motorDefaults = null;
    }
    public void initialize(ShuffleboardTab tab, TesterMotor[] motors, String name, int y) {
        entries = new GenericEntry[motors.length];
            for(int j = 0; j < motors.length; j++) {
                double max = velocityControl ? motors[j].getMaxVelocity() : 1.0;
                entries[j] = tab.add(name + ": " + motors[j].getName(), motorDefaults == null ? 0.0 : motorDefaults[j])
                    .withPosition(j * 2, y)
                    .withSize(2, 1) 
                    .withWidget(BuiltInWidgets.kNumberSlider)
                    .withProperties(Map.of("min", -max, "max", max))
                    .getEntry();
            }
    }
    public void setMotor(int motorNum, TesterMotor motor) {
        double setpoint = entries[motorNum].getDouble(motorDefaults == null ? 0.0 : motorDefaults[motorNum]);
        if(velocityControl) {
            motor.setVelocity(setpoint);
        } else {
            //System.out.println("Calling tester motor");
            motor.run(setpoint);
        }
    }
}
