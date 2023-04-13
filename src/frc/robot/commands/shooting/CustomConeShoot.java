package frc.robot.commands.shooting;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TargetHeights;
import frc.robot.subsystems.Coner;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Swerve;

public class CustomConeShoot extends CommandBase {
    private Coner coneShooter;
    private Swerve drivetrain;
    private double low;
    private double high;
    private double elevatorHeight;
    private double time;
    private Timer timer;
    private boolean hasBeenReady;
    private Lights lights;
    public CustomConeShoot(Coner coneShooter, Swerve drivetrain, double low, double high, double elevatorHeight, double time, Lights lights) {
        this.coneShooter = coneShooter;
        this.drivetrain = drivetrain;
        this.low = low;
        this.high = high;
        this.elevatorHeight = elevatorHeight;
        this.lights = lights;
        hasBeenReady = false;
        this.time = time;
        this.timer = new Timer();
        addRequirements(coneShooter.subsystem());
    }
    public void initialize() {
        if(time != 0.0) {
            timer.reset();
            timer.stop();
        }
        hasBeenReady = false;
        lights.setColor(0, 0, 255);
        coneShooter.resetTimestamp();
        coneShooter.elevator.state = elevatorHeight;
    }
    public void execute() {
        if(coneShooter.donePrepping(TargetHeights.MID)) {
            hasBeenReady = true;
            if(time != 0.0) {
                timer.reset();
                timer.start();
            }
        }
       if(hasBeenReady) {
            coneShooter.runPercentOutput(0, low);
            coneShooter.runPercentOutput(1, high);
       } else {
            coneShooter.run(TargetHeights.INTAKE);
       }
    }
    public boolean isFinished() {
        return time != 0.0 && timer.get() >= time;
    }
    public void end(boolean interrupted) {
            coneShooter.stop(TargetHeights.MID);
        if(time != 0.0) {
            timer.stop();
        }
        lights.off(drivetrain);
    }
}
