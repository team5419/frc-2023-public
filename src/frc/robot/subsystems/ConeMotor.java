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
    public ConeMotor(ShuffleboardLayout layout, boolean falcon, int port, String title) {
        if(falcon) {
            neo = null;
            talon = new TalonFX(port);
            Util.setUpMotor(talon);
        } else {
            talon = null;
            neo = new CANSparkMax(port, MotorType.kBrushless);
            Util.setUpMotor(neo, false, false);
        }
        intake = getEntry(layout, title + " Intake speed", 0);
        outtakeMid = getEntry(layout, title + " Outtake mid", 1);
        outtakeHigh = getEntry(layout, title + " Outtake high", 2);
    }
    private GenericEntry getEntry(ShuffleboardLayout layout, String title, int y) {
        return layout.add(title, 0.0)
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
        set(intake.getDouble(0.0));
    }
    public void outtakeMid() {
        set(outtakeMid.getDouble(0.0));
    }
    public void outtakeHigh() {
        set(outtakeHigh.getDouble(0.0));
    }
    public void stop() {
        set(0.0);
    }
}