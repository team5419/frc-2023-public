package frc.robot.commands; // auto align using only limelight 
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.Constants.Limelight;
public class AutoAlign extends CommandBase {
    private Swerve drivetrain;
    private Vision vision;
    private boolean shouldFinish;
    public AutoAlign(Swerve drivetrain, Vision vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        shouldFinish = false;
        addRequirements(drivetrain);
    }
    public void initialize() {
        
    }
    public void execute() {
        double turnDiff = Limelight.desiredAngle - drivetrain.angle();
        double turn = Limelight.turnPID.calculate(turnDiff);
        double leftDiff = -vision.getHorizontalOffset();
        double left = Limelight.horizontalPID.calculate(leftDiff);
        double forwardDiff = Limelight.desiredDistance - vision.getHorizontalDistance();
        double forward = Limelight.forwardPID.calculate(forwardDiff);
        drivetrain.drive(forward, left, turn, false, true);

        shouldFinish = Math.abs(turnDiff) < Limelight.epsilonTurn && Math.abs(forwardDiff) < Limelight.epsilonForward && Math.abs(leftDiff) < Limelight.epsilonHorizontal;
    }
    public boolean isFinished() {
        return shouldFinish;
    }
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}