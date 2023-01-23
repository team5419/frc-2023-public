package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.Ports;
import frc.robot.Util;

public class Intake extends SubsystemBase { // basic intake using a single neo
    private CANSparkMax motor; // main neo motor
    public Intake() {
        motor = new CANSparkMax(Ports.intake, MotorType.kBrushless); // instantiate motor using predefined port number
        Util.setUpMotor(motor, true, false); // set up the motor, it happens to be inverted
    }
    public void run(double percent) { // set the motor to a given percent output
        motor.set(percent);
    }
    public void periodic() {

    }
    public void simulationPeriodic() {

    }
}