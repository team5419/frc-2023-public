package frc.robot.subsystems;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.Ports;
import frc.robot.Util;

public class Intake extends SubsystemBase {
    private CANSparkMax motor;
    private GenericEntry widget;
    public Intake(ShuffleboardTab tab) {
        motor = new CANSparkMax(Ports.intake, MotorType.kBrushless);
        Util.setUpMotor(motor, false, false);
        tab.addNumber("Intake speed", () -> getVelocity());
        widget = tab.add("outtake speeed", -1.0)
            .withPosition(0, 0)
            .withSize(2, 1)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -1.0, "max", 1.0))
            .getEntry();
    }
    public void run() { // set the motor to a given percent output
        motor.set(widget.getDouble(-1.0));
    }
    public void run(double speed) {
        motor.set(speed);
    }
    public void stop() {
        motor.set(0.0);
    }
    public double getVelocity() {
        return motor.getEncoder().getVelocity();
    }
    public void periodic() {

    }
    public void simulationPeriodic() {

    }
}