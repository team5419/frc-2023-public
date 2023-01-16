package frc.robot.commands;
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
    public RunIntake(Intake intake, double speed, double time) {
        this.intake = intake;
        this.speed = speed;
        this.time = time;
        timer = new Timer();
        addRequirements(intake);
    }
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
        intake.run(speed);
    }
    public void execute() {
    }
    public boolean isFinished() {
        return time != 0.0 && timer.get() >= time;
    }
    public void end(boolean interrupted) {
        intake.run(0.0);
    }
}
