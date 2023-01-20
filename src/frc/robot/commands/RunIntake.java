package frc.robot.commands;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class RunIntake extends CommandBase {
    private Intake intake;
    private double speed;
    private double time;
    private Timer timer;
    private ShuffleboardTab tab;

    double kMeasuredPosHorizontal;
    double kTicksPerDegree;
    double currentPos;
    double degrees;
    /*public RunIntake(Intake intake, double speed, double time) {
        this.intake = intake;
        this.speed = speed;
        this.time = time;
        timer = new Timer();
        addRequirements(intake);
    }*/
    public RunIntake(Intake intake, double _speed) {
        this.intake = intake;
        speed = _speed;
        time = 0.0;
        timer = new Timer();
    }
    public void initialize() {
        if(time != 0.0) {
            timer.reset();
            timer.start();
        }
        //intake.run(speed);

        // intake.run(speed);
        kMeasuredPosHorizontal = intake.motor.getSelectedSensorPosition();
    }
    public void execute() {
        
        //kMeasuredPosHorizontal = 840; //Position measured when arm is horizontal
        kTicksPerDegree = 4096 / 360; //Sensor is 1:1 with arm rotation
        currentPos = intake.motor.getSelectedSensorPosition();
        degrees = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree;
        degrees = degrees % 360;
        //intake.motor.setSelectedSensorPosition(0.0);

        System.out.println(kMeasuredPosHorizontal);
        if (degrees > -40) {
            //intake.run(speed);
        }
        if (degrees < -50) {
            //intake.run(-speed);
        }
        intake.motor.set(ControlMode.Position, kMeasuredPosHorizontal + 3600.0 * kTicksPerDegree);
    }
    public boolean isFinished() {
        return time != 0.0 && timer.get() >= time;
    }
    public void end(boolean interrupted) {
        intake.run(0.0);
    }
}
