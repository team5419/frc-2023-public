package frc.robot.commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants.Drive;
import frc.robot.Util;
import edu.wpi.first.math.geometry.Pose2d;

public class RamseteSwerve extends CommandBase {
    private Swerve drivetrain;
    private boolean isFinished;
    private Pose2d goal;
    public RamseteSwerve(Swerve drivetrain, Pose2d goal) {
        this.drivetrain = drivetrain;
        this.goal = goal
        isFinished = false;
        addRequirements(drivetrain);
    }
    public void initialize() {
        
    }
    public void execute() { 
        double theta = drivetrain.angle();
        double targetRotation = goal.getRotation().getDegrees();
        double target = Math.round((theta - targetRotation) / 360) * 360 + targetRotation;
        double pose = drivetrain.pose();

        double xdiff = goal.getX() - pose.getX();
        double ydiff = goal.getY() - pose.getY();

        double dx = Drive.pXY * Util.deadband(xdiff, Drive.epsilonXY);
        double dy = Drive.pXY * Util.deadband(ydiff, Drive.epsilonXY);
        double magnitude = Math.sqrt(Math.pow(dx, 2.0) + Math.pow(dy, 2.0));

        if(magnitude > drivetrain.maxVelocity) {
            dx *= (drivetrain.maxVelocity / magnitude); // straight line ? idk
            dy *= (drivetrain.maxVelocity / magnitude);
        }

        double dtheta = -1 * DriveConstants.pTheta * (Math.PI / 180.0) * Util.deadband(target - theta, Drive.epsilonTheta);

        //System.out.println("theta: ${DriveConstants.pTheta * (Math.PI / 180) * (target - theta)}");
        drivetrain.drive(dx, dy, dtheta);

        shouldFinish = dx == 0 && dy == 0 && dtheta == 0 && drivetrain.getAverageSpeed() < 0.1;
    }
    public boolean isFinished() {
        return shouldFinish;
    }
    public void end(boolean interrupted) {
        drivetrain.drive(0.0, 0.0, 0.0);
    }
}