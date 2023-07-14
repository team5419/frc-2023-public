package frc.robot.commands.driving;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class MessyRamsete extends CommandBase {
    private Swerve drivetrain;
    private double maxSpeed;
    private Vision vision;
    private double tX;
    private double tY;
    private double tT;
    private int xdir;
    private int ydir;
    private int tdir;
    public MessyRamsete(Swerve drivetrain, Vision vision, Pose2d goal, double maxSpeed) {
        this.drivetrain = drivetrain;
        this.tX = goal.getX();
        this.tY = goal.getY();
        this.tT = goal.getRotation().getDegrees();
        this.vision = vision;
        xdir = 0;
        ydir = 0;
        tdir = 0;
        this.maxSpeed = maxSpeed;
        addRequirements(drivetrain);
    }
    public void initialize() {
        if(vision.team() == Alliance.Red) {
            tY *= -1;
        }
        double currentTheta = drivetrain.angle();
        tT = Math.round((currentTheta - tT) / 360.0) * 360.0 + tT;
        Pose2d current = drivetrain.pose();
        xdir = (int)Math.round(Math.signum(tX - current.getX()));
        ydir = (int)Math.round(Math.signum(tY - current.getY()));
        tdir = (int)Math.round(Math.signum(tT - currentTheta));
    }
    public void execute() {
        double theta = drivetrain.angle();
        Pose2d pose = drivetrain.pose();
        double dx = 0.0;
        double dy = 0.0;
        double dtheta = 0.0;
        if(tX > pose.getX() == xdir < 0) {
            xdir = 0;
        } else if(xdir != 0) {
            dx = tX - pose.getX();
        }
        if(tY > pose.getY() == ydir < 0) {
            ydir = 0;
        } else if(ydir != 0) {
            dy = tY - pose.getY();
        }
        if(tT > theta == tdir < 0) {
            tdir = 0;
        } else if(tdir != 0) {
            dtheta = tT - theta;
        }
        double magnitude = Math.sqrt(Math.pow(dx, 2.0) + Math.pow(dy, 2.0));
        if(magnitude > 0.0) {
            dx *= (maxSpeed / magnitude); // straight line ? idk
            dy *= (maxSpeed / magnitude);
        }  
        drivetrain.drive(dx , dy , dtheta, false, true, true);
    }
    public boolean isFinished() {
        return xdir == 0 && ydir == 0 && tdir == 0;
    }
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}