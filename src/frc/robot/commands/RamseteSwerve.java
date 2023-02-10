package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.Util;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.SwerveDriveConstants;
import edu.wpi.first.math.geometry.Pose2d;

public class RamseteSwerve extends CommandBase {
    protected Swerve drivetrain;
    protected boolean isFinished;
    protected Pose2d goal;
    protected boolean teamRelative;
    protected Vision vision;
    public RamseteSwerve(Swerve drivetrain, Vision vision, Pose2d goal, boolean teamRelative) {
        this.drivetrain = drivetrain;
        this.teamRelative = teamRelative;
        this.goal = goal;
        this.vision = vision;
        isFinished = false;
        addRequirements(drivetrain);
    }
    public void initialize() {
        System.out.println("initializing");
        if(teamRelative && vision.team == Vision.Team.RED) {
            this.goal = new Pose2d(AprilTagConstants.totalX - goal.getX(), AprilTagConstants.totalY - goal.getY(), goal.getRotation());
        }
    }
    public void execute() { 
        if(goal == null) {
            System.out.println("goal is null");
            return;
        }
        double theta = drivetrain.angle();
        double targetRotation = goal.getRotation().getDegrees();
        double target = Math.round((theta - targetRotation) / 360.0) * 360.0 + targetRotation;
        Pose2d pose = drivetrain.pose();

        double xdiff = goal.getX() - pose.getX();
        //System.out.println(xdiff);
        double ydiff = goal.getY() - pose.getY();
        //System.out.println(ydiff);

        double dx = SwerveDriveConstants.pXY * Util.deadband(xdiff, SwerveDriveConstants.epsilonXY);
        double dy = SwerveDriveConstants.pXY * Util.deadband(ydiff, SwerveDriveConstants.epsilonXY);
        double magnitude = Math.sqrt(Math.pow(dx, 2.0) + Math.pow(dy, 2.0));

        if(magnitude > SwerveDriveConstants.maxVelocity) {
            dx *= (SwerveDriveConstants.maxVelocity / magnitude); // straight line ? idk
            dy *= (SwerveDriveConstants.maxVelocity / magnitude);
        }

        double dtheta = 1 * SwerveDriveConstants.pTheta * (Math.PI / 180.0) * Util.deadband(target - theta, SwerveDriveConstants.epsilonTheta);
        System.out.println(dtheta);
        //System.out.println("theta: ${DriveConstants.pTheta * (Math.PI / 180) * (target - theta)}");
        drivetrain.drive(dx, dy, dtheta, true, true);

        isFinished = dx == 0 && dy == 0 && dtheta == 0 && drivetrain.getAverageSpeed() < 0.4;
    }
    public boolean isFinished() {
        System.out.println("finished");
        return isFinished;
    }
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}