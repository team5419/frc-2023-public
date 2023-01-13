package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants.Ports;
import frc.robot.Util;
import frc.robot.Constants.ClawConstants;

public class Claw extends TwoPhaseSubsystem {
    private TalonFX motor;
    public Claw() {
        super();
        motor = new TalonFX(Ports.claw);
        Util.setUpMotor(motor, ClawConstants.PID, true, 1.0);
    }
    public void _run() {
        motor.set(ControlMode.Position, on ? ClawConstants.openPosition : ClawConstants.closedPosition);
    }
    public void periodic() {

    }
    public void simulationPeriodic() {

    }
}