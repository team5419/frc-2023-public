package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
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
    public CommandBase twoPhase() {
        return Commands.runOnce(() -> this.run());
    }
}