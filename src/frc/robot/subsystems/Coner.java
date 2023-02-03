package frc.robot.subsystems;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.ConerConstants;
import frc.robot.Constants.Ports;
import frc.robot.subsystems.test.TesterFalcon;
import frc.robot.subsystems.test.TesterMotor;
import frc.robot.subsystems.test.TesterNeo;
import frc.robot.subsystems.test.TesterSetting;
import frc.robot.subsystems.test.TesterSubsystem;
import frc.robot.Util;

public class Coner extends TesterSubsystem implements GenericShootIntake {
    public Coner(boolean falcons) {
        super("Cone Shooter", new TesterMotor[] {
            generateTesterMotor("Low motor", falcons, Ports.coneBottom),
            generateTesterMotor("High motor", falcons, Ports.coneTop)
        }, Map.of("low", new TesterSetting(new double[] { ConerConstants.defaultLowLow, ConerConstants.defaultTopLow }),
                  "mid", new TesterSetting(new double[] { ConerConstants.defaultLowMid, ConerConstants.defaultTopMid }),
                  "high", new TesterSetting(new double[] { ConerConstants.defaultLowHigh, ConerConstants.defaultTopHigh }),
                  "intake", new TesterSetting(new double[] { ConerConstants.defaultLowIntake, ConerConstants.defaultTopIntake })));
    }
    public static TesterMotor generateTesterMotor(String name, boolean falcons, int id) {
        return falcons ? new TesterFalcon(name, Util.setUpMotor(new TalonFX(id)))
        : new TesterNeo(name, Util.setUpMotor(new CANSparkMax(id, MotorType.kBrushless), false, false));
    }
    public void shootHigh() { // set the motor to a given percent output
        run("high");
    }
    public void shootMid() {
        run("mid");
    }
    public void shootLow() {
        run("low");
    }
    public void intake() {
        run("intake");
    }
    public void stop() {
        super.stop();
    }
    public void setup(int a) {}
    public void periodic() {

    }
    public void simulationPeriodic() {

    }
}