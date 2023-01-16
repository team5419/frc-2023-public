package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants.ProtoDrive;

public class StupidRamsete extends CommandBase {
    private Drivetrain drivetrain;
    private double rotation;
    private double forward;
    private Timer timer;
    private boolean turnInverted;
    private boolean throttleInverted;

    public StupidRamsete(Drivetrain drivetrain, double rotation, double forward, boolean turnInverted, boolean throttleInverted){
        this.drivetrain = drivetrain;
        this.rotation = rotation;
        this.forward = forward;
        this.turnInverted = turnInverted;
        this.throttleInverted = throttleInverted;
        this.timer = new Timer();
        addRequirements(drivetrain);
    }

    public void initialize() {
        timer.reset();
        timer.start();
    }
    public void execute() {
        double time = timer.get();
        if(time < rotation) {
            drivetrain.drive(0.0, turnInverted ? (-ProtoDrive.stupidTurn) : ProtoDrive.stupidTurn);
        } else {
            //System.out.println(throttleInverted ? (-ProtoDrive.stupidThrottle) : ProtoDrive.stupidThrottle);
            drivetrain.drive(throttleInverted ? (ProtoDrive.stupidThrottle) : -ProtoDrive.stupidThrottle, 0.0);
        }
    }
    public boolean isFinished() {
        return timer.get() > rotation + forward;
    }
    public void end(boolean interrupted) {
        drivetrain.drive(0.0, 0.0);
        timer.stop();
    }
}
