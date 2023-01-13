package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants.Ports;
import frc.robot.Util;
import frc.robot.classes.PID;

public class Intake extends SubsystemBase {
    private TalonFX motor;
    public Intake() {
        motor = new TalonFX(Ports.intake);
        Util.setUpMotor(motor, new PID(0.0, 0.0, 0.0), false, 0.1);
    }
    public void run(double percent) {
        motor.set(ControlMode.PercentOutput, percent);
    }
    public void periodic() {

    }
    public void simulationPeriodic() {

    }
}