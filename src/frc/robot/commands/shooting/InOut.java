package frc.robot.commands.shooting;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Coner;
import frc.robot.Constants.TargetHeights;
public class InOut extends CommandBase {
    private Coner shooter;
    private Timer timer;
    private final double inTime = 1.0;
    private final double outTime = 0.4;
    public InOut(Coner shooter) {
        this.shooter = shooter;
        timer = new Timer();
        addRequirements(shooter.subsystem());
    }
    public void initialize() {
        timer.reset();
        timer.start();
    }
    public void execute() {
        shooter.inOut((timer.get() % (inTime + outTime)) <= inTime);
    }
    public boolean isFinished() {
        return false;
    }
    public void end(boolean interrupted) {
        shooter.stop(TargetHeights.INTAKE);
        timer.stop();
    }
}