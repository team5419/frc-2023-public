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

<<<<<<< HEAD
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
=======
public class ConeMotor { // simplify the way we store each motor for the cone intake/shooter
    private CANSparkMax neo; // if the motor is a neo, store it here
    private TalonFX talon; // otherwise, if the motor is a falcon, store it here
    private GenericEntry intake; // store a shuffleboard entry so we can keep track of a user-controlled slider value to control intake speed
    private GenericEntry outtakeMid; // same thing, for the outtake speed shooting on the mid pole
    private GenericEntry outtakeHigh; // same, for the outtake speed shooting on the high pole
    public ConeMotor(ShuffleboardLayout layout, boolean falcon, int port, String title) {
        if(falcon) { // if the cone motor runs on a falcon, set up the motor accordingly
>>>>>>> e1671c2ed58933bd695d6d35ddbaf33607ca688c
            neo = null;
            talon = new TalonFX(port);
            Util.setUpMotor(talon);
        } else { // otherwise, set it up like a neo
            talon = null;
            neo = new CANSparkMax(port, MotorType.kBrushless);
            Util.setUpMotor(neo, false, false);
        }
<<<<<<< HEAD
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
=======
        intake = getEntry(layout, title + " Intake speed", 0); // get shuffleboard entries for each of the three sliders
        outtakeMid = getEntry(layout, title + " Outtake mid", 1); // the strings here represent the title of each slider
        outtakeHigh = getEntry(layout, title + " Outtake high", 2);
    }
    private GenericEntry getEntry(ShuffleboardLayout layout, String title, int y) { // helper function for setting up sliders
        return layout.add(title, 0.0) // add a new number value
            .withPosition(0, y) // place it on the far left of the container at a given height
            .withSize(2, 1) // 2 units wide, 1 unit tall
            .withWidget(BuiltInWidgets.kNumberSlider) // give it a number slider widget
            .withProperties(Map.of("min", -1.0, "max", 1.0)) // the number slider should go from -1 to 1
            .getEntry(); // return an entry that allows us to access the current value of the slider
>>>>>>> e1671c2ed58933bd695d6d35ddbaf33607ca688c
    }
    private void set(double v) { // helper function for setting the motor to a value
        if(neo == null) { // if it's a falcon, call accordingly
            talon.set(ControlMode.PercentOutput, v);
        } else { // otherwise set the neo motor to the output percent
            neo.set(v);
        }
    }
<<<<<<< HEAD
    public void intake() {
        set(intake.getDouble(defaultIntake));
    }
    public void outtakeMid() {
        set(outtakeMid.getDouble(defaultMid));
    }
    public void outtakeHigh() {
        set(outtakeHigh.getDouble(defaultHigh));
=======
    public void intake() { // intake the motor
        set(intake.getDouble(0.0)); // pass in the value from the intake slider
    }
    public void outtakeMid() {
        set(outtakeMid.getDouble(0.0)); // rely on the outtake mid value to run the motor on outtake mid
    }
    public void outtakeHigh() {
        set(outtakeHigh.getDouble(0.0)); // same thing, using the outtake high value
>>>>>>> e1671c2ed58933bd695d6d35ddbaf33607ca688c
    }
    public void stop() { // stop the motor by setting it to 0
        set(0.0);
    }
}