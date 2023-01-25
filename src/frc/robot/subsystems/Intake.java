package frc.robot.subsystems;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.Ports;
import frc.robot.Util;

public class Intake extends SubsystemBase {
    private CANSparkMax motor;
    public Intake(ShuffleboardTab tab) {
        motor = new CANSparkMax(Ports.intake, MotorType.kBrushless);
        Util.setUpMotor(motor, false, false);
        tab.addNumber("Intake speed", () -> getVelocity());
    }
    public void run(double percent) { // set the motor to a given percent output
        motor.set(percent);
    }
    public double getVelocity() {
        return motor.getEncoder().getVelocity();
    }
    public void periodic() {

    }
    public void simulationPeriodic() {

    }
}