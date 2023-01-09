package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.TwoPhaser;
public abstract class TwoPhaseSubsystem extends SubsystemBase {
    protected boolean on;
    public TwoPhaseSubsystem() {
        on = false;
    }
    public void run() {
        on = !on;
        _run();
    };
    protected abstract void _run();
    public TwoPhaser twoPhase() {
        return new TwoPhaser(this);
    }
}