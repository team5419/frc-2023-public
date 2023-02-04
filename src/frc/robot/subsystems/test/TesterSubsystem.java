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
            i++;
        }
        for(int j = 0; j < motors.length; j++) { // display all velocities at the bottom
            int savedJ = j;
            tab.addNumber(motors[j].getName() + " VELOCITY", () -> motors[savedJ].getVelocity())
                .withSize(2, 1)
                .withPosition(j * 2, i);
        }
    }
    public void run(String setting) {
        TesterSetting theSetting = stateMap.get(setting);
        for(int i = 0; i < motors.length; i++) {
            theSetting.setMotor(i, motors[i]);
        }
    }
    protected void runSingle(String setting, int i) {
        TesterSetting theSetting = stateMap.get(setting);
        theSetting.setMotor(i, motors[i]);
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