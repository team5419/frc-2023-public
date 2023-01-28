package frc.robot.subsystems.test;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class TesterSetting {
    private double[] motorDefaults;
    private GenericEntry[] entries;
    public TesterSetting(double[] motorDefaults) {
        this.motorDefaults = motorDefaults;
    }
    public TesterSetting() {
        this.motorDefaults = null;
    }
    public void initialize(ShuffleboardTab tab, TesterMotor[] motors, String name, int x) {
        ShuffleboardLayout layout = tab.getLayout(name).withSize(2, 5).withPosition(x, 0);
        entries = new GenericEntry[motors.length];
            for(int j = 0; j < motors.length; j++) {
                entries[j] = layout.add(motors[j].getName(), motorDefaults == null ? 0.0 : motorDefaults[j])
                    .withPosition(0, j)
                    .withSize(2, 1) 
                    .withWidget(BuiltInWidgets.kNumberSlider)
                    .withProperties(Map.of("min", -1.0, "max", 1.0))
                    .getEntry();
            }
    }
    public double getValue(int motorNum) {
        return entries[motorNum].getDouble(motorDefaults == null ? 0.0 : motorDefaults[motorNum]);
    }
}
