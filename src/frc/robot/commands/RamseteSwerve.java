package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.Util;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.classes.RamseteOptions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class RamseteSwerve extends CommandBase {
    protected Swerve drivetrain;
    protected boolean isFinished;
    protected Pose2d goal;
    protected RamseteOptions options;
    protected Vision vision;
    private Alliance alliance;
    private Timer timer;
    public RamseteSwerve(Swerve drivetrain, Vision vision, Pose2d goal, RamseteOptions options) {
        this.drivetrain = drivetrain;
        this.options = options;
        this.goal = goal;
        this.vision = vision;
        isFinished = false;
        addRequirements(drivetrain);
        if(options.time != 0.0) {
            timer = new Timer();
        }
    }
    public void initialize() {
        alliance = vision.team();
        System.out.println("initializing");
        if(options.teamRelative && vision.team() == Alliance.Red) {
            this.goal = new Pose2d(goal.getX(), -goal.getY(), goal.getRotation());
        }
        if(options.time != 0.0) {
            timer.reset();
            timer.start();
        }
    }
    public void execute() { 
        if(goal == null) {
            System.out.println("goal is null");
            return;
        }
        double thetaDiff = 0.0;
        if(options.turnToTag != -1) {
            thetaDiff = vision.getHorizontalToTarget(alliance == Alliance.Blue ? (9 - options.turnToTag) : options.turnToTag);
        }
        if(options.turnToTag == -1 || !vision.seesTag) {
            double theta = drivetrain.angle();
            double targetRotation = goal.getRotation().getDegrees();
            double target = Math.round((theta - targetRotation) / 360.0) * 360.0 + targetRotation;
            thetaDiff = target - theta;
        }
        
        Pose2d pose = drivetrain.pose();

        double xdiff = goal.getX() - pose.getX();
        //System.out.println(xdiff);
        double ydiff = goal.getY() - pose.getY();
        //System.out.println(ydiff);
        double dx = options.preventDrive ? 0.0 : (SwerveDriveConstants.pXY * Util.deadband(xdiff, SwerveDriveConstants.epsilonXY * options.epsilonMultiplier));
        double dy = options.preventDrive ? 0.0 : (SwerveDriveConstants.pXY * Util.deadband(ydiff, SwerveDriveConstants.epsilonXY * options.epsilonMultiplier));
        double magnitude = Math.sqrt(Math.pow(dx, 2.0) + Math.pow(dy, 2.0));
        double max = (options.maxSpeed < 0.0 ? SwerveDriveConstants.maxVelocity : options.maxSpeed);
        if(magnitude > max) {
            dx *= (max / magnitude); // straight line ? idk
            dy *= (max / magnitude);
        }

        double dtheta = 1 * SwerveDriveConstants.pTheta * (Math.PI / 180.0) * Util.deadband(thetaDiff, SwerveDriveConstants.epsilonTheta * options.epsilonMultiplier);
        //System.out.println(dtheta);
        //System.out.println("theta: ${DriveConstants.pTheta * (Math.PI / 180) * (target - theta)}");
        drivetrain.drive(dx , dy , dtheta, true, true);

        isFinished = (options.preventDrive || (dx == 0 && dy == 0)) && (dtheta == 0 || (options.time != 0.0 && timer.get() >= options.time)) && (!options.speedLimit || drivetrain.getAverageSpeed() < 0.1) && (options.turnToTag == -1 || vision.seesTag);
    }
    public boolean isFinished() {
        System.out.println("finished");
        return isFinished;
    }
    public void end(boolean interrupted) {
        drivetrain.stop();
        if(options.time != 0.0) {
            timer.stop();
        }
    }
}