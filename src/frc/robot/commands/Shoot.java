package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TargetHeights;
import frc.robot.subsystems.GenericShootIntake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Swerve;

public class Shoot extends CommandBase {
    private GenericShootIntake coneShooter;
    private GenericShootIntake cubeShooter;
    private Swerve drivetrain;
    private boolean isCone;
    private int height;
    private double time;
    private Timer timer;
    private boolean hasBeenReady;
    private String overrideHeight;
    private Lights lights;
    private void init(GenericShootIntake coneShooter, GenericShootIntake cubeShooter, Swerve drivetrain, double time, Lights lights) {
        this.coneShooter = coneShooter;
        this.cubeShooter = cubeShooter;
        this.drivetrain = drivetrain;
        isCone = false;
        height = 0;
        this.lights = lights;
        hasBeenReady = false;
        overrideHeight = null;
        this.time = time;
        this.timer = new Timer();
        addRequirements(coneShooter.subsystem());
        addRequirements(cubeShooter.subsystem());
    }
    public Shoot(GenericShootIntake coneShooter, GenericShootIntake cubeShooter, Swerve drivetrain, Lights lights) {
        init(coneShooter, cubeShooter, drivetrain, 0.0, lights);
    }
    public Shoot(GenericShootIntake coneShooter, GenericShootIntake cubeShooter, Swerve drivetrain, double time, Lights lights) {
        init(coneShooter, cubeShooter, drivetrain, time, lights);
    }
    public Shoot(GenericShootIntake coneShooter, GenericShootIntake cubeShooter, Swerve drivetrain, double time, String overrideHeight, Lights lights) {
        init(coneShooter, cubeShooter, drivetrain, time, lights);
        this.overrideHeight = overrideHeight;
    }
    private String getHeight() {
        return overrideHeight == null ? TargetHeights.heights[height] : overrideHeight;
    }
    public void initialize() {
        if(time != 0.0) {
            timer.reset();
            timer.start();
        }
        hasBeenReady = false;
        isCone = drivetrain.usingCones;
            height = drivetrain.currentHeight; 
        lights.setColor(0, 0, 255);
    }
    public void execute() {
        System.out.println("going to shoot, height: " + height);
        GenericShootIntake shooter = isCone ? coneShooter : cubeShooter;
        String realHeight = getHeight();
       if(shooter.donePrepping(realHeight) || hasBeenReady) {
            hasBeenReady = true;
            shooter.shoot(realHeight);
       } else {
            shooter.setup(realHeight);
       }
    }
    public boolean isFinished() {
        return time != 0.0 && timer.get() >= time;
    }
    public void end(boolean interrupted) {
        String realHeight = getHeight();
        if(isCone) {
            coneShooter.stop(realHeight);
        } else {
            cubeShooter.stop(realHeight);
        }
        if(time != 0.0) {
            timer.stop();
        }
        lights.off(drivetrain);
    }
}
