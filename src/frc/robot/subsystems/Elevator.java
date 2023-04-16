package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util;
import frc.robot.Constants.Ports;
import frc.robot.Constants.TargetHeights;
import frc.robot.classes.PID;

public class Elevator extends SubsystemBase {
    private TalonFX controller;
    private static final PID PID = new PID(0.2, 0.0, 0.0);
    public static final Map<String, Double> heights = Map.of(
        TargetHeights.LOW, 0.0,
        TargetHeights.MID, 32000.2,
        TargetHeights.HIGH, 42292.0,
        TargetHeights.FAR, 0.0, 
        TargetHeights.INTAKE, 12500.0
    );
    private static final PIDController additionalPID = new PIDController(0.00005, 0, 0);
    public static final double down = 0.0;
    public double state;
    private double timestamp;
    public Elevator() {
        controller = new TalonFX(Ports.elevatorPort, "canivore");
        Util.setUpMotor(controller, false, true);
        controller.configMotionCruiseVelocity(34000.0);
        controller.configMotionAcceleration(48000.0);
        controller.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40.0, 0.0, 0.0));
        controller.config_kP(0, PID.p);
        controller.config_kI(0, PID.i);
        controller.config_kD(0, PID.d);
        state = down;
        ShuffleboardTab tab = Shuffleboard.getTab("Elevator");
        tab.addNumber("position", controller::getSelectedSensorPosition);
        tab.addNumber("setpoint", () -> state);
        timestamp = -1.0;
    }
    public void periodic() {
        double sensor = controller.getSelectedSensorPosition();
        if(state <= 5000.0 && sensor <= 2000.0) {
            controller.set(ControlMode.PercentOutput, 0.0);
            return;
        }
        if(Math.abs(sensor - state) < 4000.0) {
            if(timestamp == -1.0) {
                timestamp = Timer.getFPGATimestamp();
            }
            //controller.set(ControlMode.PercentOutput, additionalPID.calculate(sensor, state));
        } else {
            timestamp = -1.0;
            
        }
        controller.set(ControlMode.MotionMagic, state);
    }
    public boolean closeEnough() {
        return timestamp != -1.0 && Timer.getFPGATimestamp() - timestamp >= 0.2;
    }
}