package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TwoPhaseSubsystem;
public class TwoPhaser extends CommandBase {
    private TwoPhaseSubsystem sub;
    public TwoPhaser(TwoPhaseSubsystem sub) {
        this.sub = sub;
        addRequirements(sub);
    }
    public void initialize() {
        sub.run();
    }
    public void execute() {

    }
    public boolean isFinished() {
        return true;
    }
    public void end(boolean interrupted) {
    }
}
