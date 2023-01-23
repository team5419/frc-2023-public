package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
public abstract class TwoPhaseSubsystem extends SubsystemBase { // a little abstraction for subsystems that alternate between on and off
    protected boolean on;
    public TwoPhaseSubsystem() { // instantiate and store a boolean value for whether the subsystem is on
        on = false;
    }
    public void run() { // when the subsystem is run, on is inverted and the inner abstract function is called
        on = !on;
        _run();
    };
    protected abstract void _run();
    public CommandBase twoPhase() { // generate a simple command that calls run() and inverts the state of the subsystem
        return Commands.runOnce(() -> this.run());
    }
}