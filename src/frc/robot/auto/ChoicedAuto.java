package frc.robot.auto;
import java.util.Dictionary;
import java.util.Map;
import java.util.HashMap;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
public class ChoicedAuto {
    private static Map<String, GenericEntry> params = new HashMap<String, GenericEntry>();
    private static ShuffleboardTab tab = null;
    private static int lastX = 0;
    private static int lastY = 2;
    protected void registerKey(String key) {
        if(tab == null) {
            tab = Shuffleboard.getTab("Auto");
        }
        GenericEntry entry = params.get(key);
        if(entry == null) {
            SimpleWidget widget = tab.add(key + "?", false)
                .withSize(2, 1)
                .withPosition(lastX, lastY)
                .withWidget(BuiltInWidgets.kToggleSwitch);
            params.put(key, widget.getEntry());
            lastX += 2;
            if(lastX >= 8) {
                lastX = 0;
                lastY++;
            }
        }
    }
    protected boolean getKey(String key) {
        GenericEntry entry = params.get(key);
        if(entry == null) {
            return false;
        }
        return entry.getBoolean(false);
    }
    protected void handle(RobotContainer container, SequentialCommandGroup group) {}
    public SequentialCommandGroup evaluate(RobotContainer container) {
        SequentialCommandGroup group = new SequentialCommandGroup();
        handle(container, group);
        return group;
    }
}