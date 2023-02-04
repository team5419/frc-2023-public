package frc.robot.commands; // auto align using only limelight 
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
public class AutoAlign extends CommandBase {
    private Swerve drivetrain;
    private Vision vision;
    private boolean shouldFinish;
    private double distance;
    public AutoAlign(Swerve drivetrain, Vision vision, double distance) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.distance = distance;
        shouldFinish = false;
        addRequirements(drivetrain);
    }
    public void initialize() {
        
    }
    public void execute() {
        double turnDiff = LimelightConstants.desiredAngle - drivetrain.angle(); //calculates how many degrees to turn
        double turn = LimelightConstants.turnPID.calculate(turnDiff); //calculates amt to turn
        double leftDiff = -vision.getHorizontalOffset(); //how far sidewyas to move across fied
        double left = LimelightConstants.horizontalPID.calculate(leftDiff);// ''
        double forwardDiff = distance - LimelightConstants.desiredDistance - vision.getHorizontalDistance(); //how far forward to go
        double forward = LimelightConstants.forwardPID.calculate(forwardDiff);
        drivetrain.drive(forward, left, turn, false, true);

        shouldFinish = Math.abs(turnDiff) < LimelightConstants.epsilonTurn && Math.abs(forwardDiff) < LimelightConstants.epsilonForward && Math.abs(leftDiff) < LimelightConstants.epsilonHorizontal;
    }
    public boolean isFinished() {
        return shouldFinish;
    }
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}