package frc.robot.subsystems;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConerConstants;

import frc.robot.Constants.Ports;

public class Coner extends SubsystemBase { // stores two cone motors, one top and one bottom
    public ConeMotor top; // top and bottom conemotors
    public ConeMotor bottom;
    public Coner(ShuffleboardTab tab, boolean falcons) { // create the coner with a given shuffleboard tab, and tell it whether to use falcons or neos
        ShuffleboardLayout topLayout = tab.getLayout("Top Cone", BuiltInLayouts.kList).withPosition(3, 0).withSize(2, 5);
        ShuffleboardLayout bottomLayout = tab.getLayout("Bottom Cone", BuiltInLayouts.kList).withPosition(5, 0).withSize(2, 5);
        top = new ConeMotor(topLayout, falcons, Ports.coneTop, "Top", ConerConstants.defaultTopIntake, ConerConstants.defaultTopMid, ConerConstants.defaultTopHigh);
        bottom = new ConeMotor(bottomLayout, falcons, Ports.coneBottom, "Bottom", ConerConstants.defaultLowIntake, ConerConstants.defaultLowMid, ConerConstants.defaultLowHigh);
    }
    public void periodic() {

    }
    public void simulationPeriodic() {

    }
}