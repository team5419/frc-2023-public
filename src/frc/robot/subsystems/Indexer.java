package frc.robot.subsystems;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.Ports;
import frc.robot.Util;

public class Indexer extends SubsystemBase {
    private CANSparkMax motor;
    private GenericEntry widget;
    public Indexer(ShuffleboardTab tab) {
        motor = new CANSparkMax(Ports.indexer, MotorType.kBrushless);
        Util.setUpMotor(motor, true, false);
        widget = tab.add("outtake speeed", -0.6)
            .withPosition(0, 0)
            .withSize(2, 1)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -1.0, "max", 1.0))
            .getEntry();
    }
    public void run() {
        motor.set(widget.getDouble(-0.6));
    }
    public void run(double speed) {
        motor.set(speed);
    }
    public void stop() {
        motor.set(0.0);
    }
    public void periodic() {

    }
    public void simulationPeriodic() {

    }
}