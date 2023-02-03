package frc.robot;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.classes.PID;
import frc.robot.Constants.AprilTags;

import frc.robot.Constants.Drive;
import frc.robot.Constants.ProtoDrive;
public class Util {
    public static int getSection(double xpos) {
        if(xpos <= AprilTags.yOutsideRightChargingStation) {
            return 0;
        }
        if(xpos >= AprilTags.yInsideRightChargingStation && xpos <= AprilTags.yInsideLeftChargingStation) {
            return 1;
        }
        if(xpos >= AprilTags.yOutsideLeftChargingStation) {
            return 2;
        }
        return -1;
    }
    public static double nativeUnitsToMetersPerSecond(double units) {
        return (units / Drive.ticksPerRotation) * Drive.wheelCircumference * 10.0;
    }
    public static double nativeUnitsToMeters(double units) {
        return (units / Drive.ticksPerRotation) * Drive.wheelCircumference / Drive.unitsPerMeter;
    }
    public static double nativeUnitsToMetersProto(double units) {
        return (units / ProtoDrive.ticksPerRotation) * Drive.wheelCircumference;
    }
    public static double metersPerSecondToNativeUnits(double units) {
        return ((units / Drive.wheelCircumference) * Drive.ticksPerRotation) / 10.0;
    }
    public static double metersPerSecondToNativeUnitsProto(double units) {
        return ((units / Drive.wheelCircumference) * ProtoDrive.ticksPerRotation) / 10.0;
    }
    public static double degreesToNativeUnits(double units) {
        return (units / 360.0) * Drive.turnTicksPerRotation;
    }
    public static double deadband(double value, double tolerance) {
        return (Math.abs(value) <= tolerance) ? 0.0 : value;
    }
    public static TalonFX setUpMotor(TalonFX motor) {
        return setUpMotor(motor, false, false);
    }
    public static TalonFX setUpMotor(TalonFX motor, boolean sensorPhase, boolean inverted) {
        return setUpMotor(motor, sensorPhase, inverted, new PID(0.0, 0.0, 0.0), false, 1.0);
    }
    public static TalonFX setUpMotor(TalonFX motor, PID pid, boolean brake, double maxOutput) {
        return setUpMotor(motor, false, false, pid, brake, maxOutput);
    }
    public static TalonFX setUpMotor(TalonFX motor, boolean sensorPhase, boolean inverted, PID pid, boolean brake, double maxOutput) {
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
        return motor;
    }
    public static CANSparkMax setUpMotor(CANSparkMax motor, boolean inverted, boolean brake) {
        motor.restoreFactoryDefaults();
        motor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
        motor.setInverted(inverted);
        motor.setSmartCurrentLimit(40);
        motor.setClosedLoopRampRate(1.0);
        motor.setControlFramePeriodMs(50);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);
        return motor;
    }
}