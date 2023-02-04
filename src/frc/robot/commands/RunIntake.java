package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GenericShootIntake;

public class RunIntake extends CommandBase {
    private GenericShootIntake shooter;

    public RunIntake(GenericShootIntake shooter) {
        this.shooter = shooter;
    }

    public void initialize() {
        shooter.setup(-1);// -1 indicates that we're intaking, not shooting
    }
    public void execute() {
        shooter.intake();
    }
    public boolean isFinished() {
        return false;
    }
    public void end(boolean interrupted) {
        shooter.stop();
    }
}
