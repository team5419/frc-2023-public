package frc.robot.commands.shooting;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TargetHeights;
import frc.robot.subsystems.Cuber;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Swerve;

public class CustomShoot extends CommandBase {
    private Cuber cubeShooter;
    private Swerve drivetrain;
    private double setpoint;
    private double time;
    private Timer timer;
    private boolean hasBeenReady;
    private Lights lights;
    public CustomShoot(Cuber cubeShooter, Swerve drivetrain, double setpoint, double time, Lights lights) {
        this.cubeShooter = cubeShooter;
        this.drivetrain = drivetrain;
        this.setpoint = setpoint;
        this.lights = lights;
        hasBeenReady = false;
        this.time = time;
        this.timer = new Timer();
        addRequirements(cubeShooter.subsystem());
    }
    public void initialize() {
        if(time != 0.0) {
            timer.reset();
            timer.stop();
        }
        hasBeenReady = false;
        lights.setColor(0, 0, 255);
        cubeShooter.state = cubeShooter.shotSetpoint;
    }
    public void execute() {
        if(cubeShooter.donePrepping(setpoint)) {
            hasBeenReady = true;
            if(time != 0.0) {
                timer.reset();
                timer.start();
            }
        }
       if(hasBeenReady) {
            cubeShooter.runPercentOutput(0, 1.0);
            cubeShooter.runVelocity(1, setpoint);
       } else {
            cubeShooter.setup(setpoint);
       }
    }
    public boolean isFinished() {
        return time != 0.0 && timer.get() >= time;
    }
    public void end(boolean interrupted) {
            cubeShooter.stop(TargetHeights.MID);
        if(time != 0.0) {
            timer.stop();
        }
        lights.off(drivetrain);
    }
}
