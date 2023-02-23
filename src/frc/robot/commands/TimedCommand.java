package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TimedCommand extends CommandBase {
    private Timer timer;
    private double time;
    private Runnable _execute;
    private Runnable _stop;
    public TimedCommand(double time, Runnable execute, Runnable stop) {
        this.timer = new Timer();
        this._execute = execute;
        this._stop = stop;
        this.time = time;
    }
    public void initialize() {
        timer.reset();
        timer.start();
    }
    public void execute() {
        this._execute.run();
    }
    public void stop(boolean interrupted) {
        timer.stop();
        this._stop.run();
    }
    public boolean isFinished() {
        return timer.get() >= time;
    }
}
