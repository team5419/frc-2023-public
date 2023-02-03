package frc.robot.subsystems.test;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Map;

public class TesterSubsystem extends SubsystemBase {
    private Map<String, TesterSetting> stateMap;
    protected TesterMotor[] motors;
    public TesterSubsystem(String name, TesterMotor[] motors, Map<String, TesterSetting> states) {
        ShuffleboardTab tab = Shuffleboard.getTab(name);
        this.motors = motors;
        this.stateMap = states;
        int i = 0;
        for(Map.Entry<String, TesterSetting> entry : states.entrySet()) {
            entry.getValue().initialize(tab, motors, entry.getKey(), i);
            i += 2;
        }
    }
    public void run(String setting) {
        TesterSetting theSetting = stateMap.get(setting);
        for(int i = 0; i < motors.length; i++) {
            motors[i].run(theSetting.getValue(i));
        }
    }
    protected void stop() {
        for(int i = 0; i < motors.length; i++) {
            motors[i].run(0.0);
        }
    }
    public void periodic() {

    }
    public void simulationPeriodic() {

    }
}