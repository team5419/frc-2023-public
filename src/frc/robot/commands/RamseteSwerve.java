package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants.Drive;
import frc.robot.Util;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class RamseteSwerve extends CommandBase {
    protected Swerve drivetrain;
    protected boolean isFinished;
    protected Pose2d goal;
    public RamseteSwerve(Swerve drivetrain, Pose2d goal) {
        this.drivetrain = drivetrain;
        this.goal = goal;
        isFinished = false;
        addRequirements(drivetrain);
    }
    public RamseteSwerve(Swerve drivetrain, double changeX, double changeY, double changeTheta) { // from current position
        
        // ^ finds coordinates of the current position + the change in coordinates
        isFinished = false;
        addRequirements(drivetrain);
    }
    public void initialize() {
        System.out.println("initializing");
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

        double dx = Drive.pXY * Util.deadband(xdiff, Drive.epsilonXY);
        double dy = Drive.pXY * Util.deadband(ydiff, Drive.epsilonXY);
        double magnitude = Math.sqrt(Math.pow(dx, 2.0) + Math.pow(dy, 2.0));

        if(magnitude > Drive.maxVelocity) {
            dx *= (Drive.maxVelocity / magnitude); // straight line ? idk
            dy *= (Drive.maxVelocity / magnitude);
        }

        double dtheta = 1 * Drive.pTheta * (Math.PI / 180.0) * Util.deadband(target - theta, Drive.epsilonTheta);
        System.out.println(dtheta);
        //System.out.println("theta: ${DriveConstants.pTheta * (Math.PI / 180) * (target - theta)}");
        drivetrain.drive(dx, dy, dtheta, true, true);

        isFinished = dx == 0 && dy == 0 && dtheta == 0 && drivetrain.getAverageSpeed() < 0.1;
    }
    public boolean isFinished() {
        System.out.println("finished");
        return isFinished;
    }
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}