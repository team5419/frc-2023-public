package frc.robot.subsystems.test;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Map;

public class TesterSubsystem extends SubsystemBase {
    public static boolean usingShuffleboard = false;
    private Map<String, TesterSetting[]> stateMap;
    protected TesterMotor[] motors;
    public TesterSubsystem(String name, TesterMotor[] motors, Map<String, TesterSetting[]> states) {
        ShuffleboardTab tab = Shuffleboard.getTab(name);
        this.motors = motors;
        this.stateMap = states;
        
            int i = 0;
            for(Map.Entry<String, TesterSetting[]> entry : states.entrySet()) {
                if(usingShuffleboard) {
                for(int x = 0; x < motors.length; x++) {
                    entry.getValue()[x].initialize(tab, motors[x], entry.getKey(), x, i);
                }
             }
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
        TesterSetting[] theSetting = stateMap.get(setting);
        for(int i = 0; i < theSetting.length; i++) {
            theSetting[i].setMotor(motors[i]);
        }
    }
    protected void runSingle(String setting, int i) {
        TesterSetting[] theSetting = stateMap.get(setting);
        theSetting[i].setMotor(motors[i]);
    }
    protected void stop() {
        for(int i = 0; i < motors.length; i++) {
            motors[i].stop();
        }
    }
    public void changeMotorOffset(String setting, int motorNum, double value) {
        TesterSetting[] s = stateMap.get(setting);
        s[motorNum].changeOffset(value, motors[motorNum].getMaxVelocity());
    }
    public void periodic() {

    }
    public void simulationPeriodic() {

    }
}