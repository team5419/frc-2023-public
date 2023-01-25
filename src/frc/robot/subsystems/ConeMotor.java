package frc.robot.subsystems;

import frc.robot.Util;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class ConeMotor {
    private CANSparkMax neo;
    private TalonFX talon;
    private GenericEntry intake;
    private GenericEntry outtakeMid;
    private GenericEntry outtakeHigh;
    private double defaultIntake;
    private double defaultMid;
    private double defaultHigh;
    public ConeMotor(ShuffleboardLayout layout, boolean falcon, int port, String title, double defaultIntake, double defaultMid, double defaultHigh) {
        if(falcon) {
            neo = null;
            talon = new TalonFX(port);
            Util.setUpMotor(talon);
        } else {
            talon = null;
            neo = new CANSparkMax(port, MotorType.kBrushless);
            Util.setUpMotor(neo, false, false);
        }
        this.defaultIntake = defaultIntake;
        this.defaultMid = defaultMid;
        this.defaultHigh = defaultHigh;
        intake = getEntry(layout, title + " Intake speed", 0, defaultIntake);
        outtakeMid = getEntry(layout, title + " Outtake mid", 1, defaultMid);
        outtakeHigh = getEntry(layout, title + " Outtake high", 2, defaultHigh);
    }
    private GenericEntry getEntry(ShuffleboardLayout layout, String title, int y, double def) {
        return layout.add(title, def)
            .withPosition(0, y)
            .withSize(2, 1)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -1.0, "max", 1.0))
            .getEntry();
    }
    private void set(double v) {
        if(neo == null) {
            talon.set(ControlMode.PercentOutput, v);
        } else {
            neo.set(v);
        }
    }
    public void intake() {
        set(intake.getDouble(defaultIntake));
    }
    public void outtakeMid() {
        set(outtakeMid.getDouble(defaultMid));
    }
    public void outtakeHigh() {
        set(outtakeHigh.getDouble(defaultHigh));
    }
    public void stop() {
        set(0.0);
    }
}