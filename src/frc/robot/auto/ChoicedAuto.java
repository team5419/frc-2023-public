package frc.robot.auto;
import java.util.Dictionary;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
public class ChoicedAuto extends SequentialCommandGroup {
    private static Dictionary<String, GenericEntry> params;
    private static ShuffleboardTab tab = null;
    private static int lastX = 0;
    private static int lastY = 2;
    public static void handleReqs(String[] reqs) {
        for(String str : reqs) {
            registerKey(str);
        }
    } 
    public static void registerKey(String key) {
        if(tab == null) {
            tab = Shuffleboard.getTab("Auto");
        }
        GenericEntry entry = params.get(key);
        if(entry == null) {
            SimpleWidget widget = tab.add(entry + "?", false)
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
    public boolean getKey(String key) {
        GenericEntry entry = params.get(key);
        if(entry == null) {
            return false;
        }
        return entry.getBoolean(false);
    }
    public SequentialCommandGroup setupWith(RobotContainer container) {
        return this;
    }
}