package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.Ports;
import frc.robot.Util;

public class Indexer extends SubsystemBase {
    private CANSparkMax motor;
    public Indexer() {
        motor = new CANSparkMax(Ports.indexer, MotorType.kBrushless);
        Util.setUpMotor(motor, true, false);
    }
    public void run(double percent) {
        motor.set(percent);
    }
    public void periodic() {

    }
    public void simulationPeriodic() {

    }
}