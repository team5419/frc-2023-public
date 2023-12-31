package frc.robot.commands.shooting;
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
    private double forceTimeout;
    private void init(GenericShootIntake coneShooter, GenericShootIntake cubeShooter, Swerve drivetrain, double time, Lights lights, double forceTimeout) {
        this.coneShooter = coneShooter;
        this.cubeShooter = cubeShooter;
        this.forceTimeout = forceTimeout;
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
        init(coneShooter, cubeShooter, drivetrain, 0.0, lights, 0.0);
    }
    public Shoot(GenericShootIntake coneShooter, GenericShootIntake cubeShooter, Swerve drivetrain, double time, Lights lights) {
        init(coneShooter, cubeShooter, drivetrain, time, lights, 0.0);
    }
    public Shoot(GenericShootIntake coneShooter, GenericShootIntake cubeShooter, Swerve drivetrain, double time, double forceTime, Lights lights) {
        init(coneShooter, cubeShooter, drivetrain, time, lights, forceTime);
    }
    public Shoot(GenericShootIntake coneShooter, GenericShootIntake cubeShooter, Swerve drivetrain, double time, String overrideHeight, Lights lights) {
        init(coneShooter, cubeShooter, drivetrain, time, lights, 0.0);
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
        GenericShootIntake shooter = isCone ? coneShooter : cubeShooter;
        shooter.setup(getHeight());
    }
    public void execute() {
        if(!isCone) {
            drivetrain.brake();
        }
        
        GenericShootIntake shooter = isCone ? coneShooter : cubeShooter;
        String realHeight = getHeight();
        if(shooter.donePrepping(realHeight) && !hasBeenReady) {
            hasBeenReady = true;
            if(time != 0.0) {
                timer.reset();
                timer.start();
            }
        }
       if(hasBeenReady) {
            shooter.shoot(realHeight);
       } else {
            shooter.setup(realHeight);
       }
    }
    public boolean isFinished() {
        return time != 0.0 && ((timer.get() >= time && hasBeenReady) || (forceTimeout != 0.0 && timer.get() >= forceTimeout && !hasBeenReady));
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
