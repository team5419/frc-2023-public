package frc.robot.commands; // auto align using only limelight 
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
public class AprilAlign extends CommandBase {
    private Swerve drivetrain;
    private Vision vision;
    private boolean isFinished;
    private double timeoutTime;
    private Timer timer;
    private int num;
    public AprilAlign(Swerve drivetrain, Vision vision, int num, double timeoutTime) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        addRequirements(drivetrain);
        isFinished = false;
        this.num = num;
        this.timeoutTime = timeoutTime;
        this.timer = new Timer();
    }
    public void initialize() {
        isFinished = false;
        timer.reset();
        timer.start();
    }
    public void execute() {
        double theta = vision.getHorizontalToTarget(num);
        if(Math.abs(theta) < 0.04) {
            theta = 0.0;
        }
       
        isFinished = theta == 0.0 && drivetrain.getAverageSpeed() < 0.2 && vision.seesTag;
        drivetrain.drive(0.0, 0.0, theta * 3, false, true);
    }
    public boolean isFinished() {
        return isFinished || timer.get() >= timeoutTime;
    }
    public void end(boolean interrupted) {
        drivetrain.stop();
        timer.stop();
    }
}