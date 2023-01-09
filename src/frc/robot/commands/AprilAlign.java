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
    public AprilAlign(Swerve drivetrain, Vision vision, double[] positions, double distance, int num) {
        this.drivetrain = drivetrain;
        shouldFinish = vision.team == Vision.Team.NONE;
        target = 0.0;
        this.distance = 0.0;
        if(vision.team != Vision.Team.NONE) {
            target = positions[vision.team == Vision.Team.RED ? num : (positions.length - num - 1)];
            this.distance = vision.team == Vision.Team.BLUE ? distance : (AprilTags.fieldLength - distance);
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

        shouldFinish = Math.abs(turnDiff) < AprilTags.epsilonTurn && Math.abs(forwardDiff) < AprilTags.epsilonForward && Math.abs(leftDiff) < AprilTags.epsilonHorizontal;
    }
    public boolean isFinished() {
        return shouldFinish;
    }
    public void end(boolean interrupted) {
        drivetrain.drive(0.0, 0.0, 0.0);
    }
}