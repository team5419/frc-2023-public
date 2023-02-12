package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TargetHeights;
import frc.robot.subsystems.GenericShootIntake;

public class RunIntake extends CommandBase {
    private GenericShootIntake shooter;
    private double time;
    private Timer timer;

    public RunIntake(GenericShootIntake shooter, double time) {
        this.shooter = shooter;
        this.time = time;
        this.timer = new Timer();
        addRequirements(shooter.subsystem());
    }

    public RunIntake(GenericShootIntake shooter) {
        this.shooter = shooter;
        this.time = 0.0;
        this.timer = new Timer();
        addRequirements(shooter.subsystem());
    }

    public void initialize() {
        if(time != 0.0) {
            timer.reset();
            timer.start();
        }
        shooter.setup(TargetHeights.INTAKE, true);// -1 indicates that we're intaking, not shooting
    }
    public void execute() {
        shooter.shoot(TargetHeights.INTAKE);
    }
    public boolean isFinished() {
        return time != 0.0 && timer.get() >= time;
    }
    public void end(boolean interrupted) {
        shooter.stop(TargetHeights.INTAKE);
        if(time != 0.0) {
            timer.stop();
        }
    }
}
