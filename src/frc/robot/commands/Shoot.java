package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TargetHeights;
import frc.robot.subsystems.GenericShootIntake;
import frc.robot.subsystems.Swerve;

public class Shoot extends CommandBase {
    private GenericShootIntake coneShooter;
    private GenericShootIntake cubeShooter;
    private Swerve drivetrain;
    private boolean isCone;
    private int height;
    private double time;
    private Timer timer;
    private void init(GenericShootIntake coneShooter, GenericShootIntake cubeShooter, Swerve drivetrain, double time) {
        this.coneShooter = coneShooter;
        this.cubeShooter = cubeShooter;
        this.drivetrain = drivetrain;
        isCone = false;
        height = 0;
        this.time = time;
        this.timer = new Timer();
        addRequirements(coneShooter.subsystem());
        addRequirements(cubeShooter.subsystem());
    }
    public Shoot(GenericShootIntake coneShooter, GenericShootIntake cubeShooter, Swerve drivetrain) {
        init(coneShooter, cubeShooter, drivetrain, 0.0);
    }
    public Shoot(GenericShootIntake coneShooter, GenericShootIntake cubeShooter, Swerve drivetrain, double time) {
        init(coneShooter, cubeShooter, drivetrain, time);
    }
    public void initialize() {
        if(time != 0.0) {
            timer.reset();
            timer.start();
        }
        isCone = drivetrain.currentNum != 1;
        height = drivetrain.currentHeight; 
    }
    public void execute() {
        System.out.println("going to shoot, height: " + height);
        GenericShootIntake shooter = isCone ? coneShooter : cubeShooter;
        shooter.shoot(TargetHeights.heights[height]);
    }
    public boolean isFinished() {
        return time != 0.0 && timer.get() >= time;
    }
    public void end(boolean interrupted) {
        if(isCone) {
            coneShooter.stop(TargetHeights.heights[height]);
        } else {
            cubeShooter.stop(TargetHeights.heights[height]);
        }
        if(time != 0.0) {
            timer.stop();
        }
    }
}
