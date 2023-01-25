package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants.Ports;
import frc.robot.Util;
import frc.robot.Constants.ClawConstants;

public class Claw extends TwoPhaseSubsystem { // the claw goes on and off, so we can shortcut it with twophasesubsystem
    // the main claw motor
    private TalonFX motor;
    public Claw() {
        super(); // call the inherited constructor
        motor = new TalonFX(Ports.claw); // initialize the motor using the predefined claw port
        Util.setUpMotor(motor, ClawConstants.PID, true, 1.0); // set up the motor with a certain PID, brake mode on, and a max output of 100%
    }
    public void _run() { // This function gets called by twophasesubsystem and makes use of the inherited on property
        // the on property decides whether the claw should be on
        motor.set(ControlMode.Position, on ? ClawConstants.openPosition : ClawConstants.closedPosition); // set position based on desired state
    }
    public void periodic() {

    }
    public void simulationPeriodic() {

    }
}