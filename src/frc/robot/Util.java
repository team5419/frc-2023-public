package frc.robot;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.classes.PID;

import frc.robot.Constants.Drive;
public class Util {
    public static double nativeUnitsToMetersPerSecond(double units) {
        return (units / Drive.ticksPerRotation) * Drive.wheelCircumference * 10.0;
    }
    public static double metersPerSecondToNativeUnits(double units) {
        return ((units / Drive.wheelCircumference) * Drive.ticksPerRotation) / 10.0;
    }
    public static void setUpMotor(TalonFX motor) {
        setUpMotor(motor, false, false);
    }
    public static void setUpMotor(TalonFX motor, boolean sensorPhase, boolean inverted) {
        setUpMotor(motor, sensorPhase, inverted, new PID(0.0, 0.0, 0.0), false, 1.0);
    }
    public static void setUpMotor(TalonFX motor, PID pid, boolean brake, double maxOutput) {
        setUpMotor(motor, false, false, pid, brake, maxOutput);
    }
    public static void setUpMotor(TalonFX motor, boolean sensorPhase, boolean inverted, PID pid, boolean brake, double maxOutput) {
        motor.configFactoryDefault(100);
        motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40.0, 0.0, 0.0), 100);

        motor.setSensorPhase(sensorPhase);
        motor.setInverted(inverted);

        motor.config_kP( 0, pid.p, 100 );
        motor.config_kI( 0, pid.i, 100 );
        motor.config_kD( 0, pid.d, 100 );

        motor.setSelectedSensorPosition(0.0, 0, 100);

        motor.configVoltageCompSaturation(12.0, 100);
        motor.enableVoltageCompensation(true);
        motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 50, 100);
        motor.setControlFramePeriod(ControlFrame.Control_3_General, 50);
        motor.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);

        motor.configClosedLoopPeakOutput(0, maxOutput, 100);
    }
}