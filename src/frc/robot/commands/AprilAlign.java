package frc.robot.commands; // auto align using only april tags
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.Constants.AprilTags;
public class AprilAlign extends CommandBase {
    private Swerve drivetrain;
    private boolean shouldFinish;
    private double target;
    private double distance;
    private boolean isCone;
    public AprilAlign(Swerve drivetrain, Vision vision, int num, int height, boolean isCone) {
        this.drivetrain = drivetrain;
        shouldFinish = vision.team == Vision.Team.NONE;
        target = 0.0;
        this.distance = 0.0;
        this.isCone = isCone;
        if(vision.team != Vision.Team.NONE) {
            target = AprilTags.xPositions[vision.team == Vision.Team.RED ? num : (AprilTags.xPositions.length - num - 1)]; 
            this.distance = isCone ? AprilTags.coneDists[height] : AprilTags.cubeDists[height];
        }
        addRequirements(drivetrain);
    }
    public void initialize() {
        
    }
    public void execute() {
        Pose2d pose = drivetrain.pose();
        double turnDiff = -drivetrain.angle();
        double turn = AprilTags.turnPID.calculate(turnDiff);
        double leftDiff = target - pose.getY();
        double left = AprilTags.horizontalPID.calculate(leftDiff);
        double forwardDiff = distance - pose.getX();
        double forward = AprilTags.forwardPID.calculate(forwardDiff);
        drivetrain.drive(forward, left, turn);

        shouldFinish = Math.abs(turnDiff) < (isCone ? AprilTags.coneEpsilonTurn : AprilTags.epsilonTurn) &&
         Math.abs(forwardDiff) < (isCone ? AprilTags.coneEpsilonForward : AprilTags.epsilonForward) && 
         Math.abs(leftDiff) < (isCone ? AprilTags.coneEpsilonHorizontal : AprilTags.epsilonHorizontal);
    }
    public boolean isFinished() {
        return shouldFinish;
    }
    public void end(boolean interrupted) {
        drivetrain.drive(0.0, 0.0, 0.0);
    }
}