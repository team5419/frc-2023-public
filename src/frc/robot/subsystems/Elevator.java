package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util;
import frc.robot.Constants.Ports;
import frc.robot.Constants.TargetHeights;
import frc.robot.classes.PID;

public class Elevator extends SubsystemBase {
    private TalonFX controller;
    private static final PID PID = new PID(0.5, 0.0, 0.0);
    public static final Map<String, Double> heights = Map.of(
        TargetHeights.LOW, 0.0,
        TargetHeights.MID, 0.0,
        TargetHeights.HIGH, 0.0,
        TargetHeights.FAR, 0.0, 
        TargetHeights.INTAKE, 0.0
    );
    public static final double down = 0.0;
    public double state;
    public Elevator() {
        controller = new TalonFX(Ports.elevatorPort, "canivore");
        Util.setUpMotor(controller, false, false);
        controller.configMotionCruiseVelocity(3200.0);
        controller.configMotionAcceleration(9600.0 * 4);
        controller.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40.0, 0.0, 0.0));
        controller.config_kP(0, PID.p);
        controller.config_kI(0, PID.i);
        controller.config_kD(0, PID.d);
        state = down;
        ShuffleboardTab tab = Shuffleboard.getTab("Elevator");
        tab.addNumber("position", controller::getSelectedSensorPosition);
        tab.addNumber("setpoint", () -> state);
    }
    public void periodic() {
        //controller.set(ControlMode.MotionMagic, state);
    }
    public boolean closeEnough() {
        return Math.abs(controller.getSelectedSensorPosition() - state) < 100.0;
    }
}
