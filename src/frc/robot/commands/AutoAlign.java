package frc.robot.commands; // auto align using only limelight 
import frc.robot.Util;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.TargetHeights;
import frc.robot.subsystems.GenericShootIntake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
public class AutoAlign extends CommandBase {
    private Swerve drivetrain;
    private Vision vision;
    private double distance;
    private GenericShootIntake shooter;
    private int height;
    private Lights lights;
    private boolean isFinished;
    private Timer timer;
    private double time;
    public AutoAlign(Swerve drivetrain, GenericShootIntake shooter, Vision vision, double distance, int height, Lights lights, double timeLimit) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.distance = distance;
        this.time = timeLimit;
        this.shooter = shooter;
        this.height = height;
        this.lights = lights;
        isFinished = false;
        addRequirements(drivetrain, shooter.subsystem());
        this.timer = new Timer();
    }
    public void initialize() {
        isFinished = false;
        vision.on();
        if(time != 0.0) {
            timer.reset();
            timer.start();
        }
    }
    public void execute() {
        double theta = drivetrain.angle();
        double target = Math.round((theta - LimelightConstants.desiredAngle) / 360.0) * 360.0 + LimelightConstants.desiredAngle;
        double turnDiff = Util.deadband(target - drivetrain.angle(), LimelightConstants.epsilonTurn); //calculates how many degrees to turn
        double turn = LimelightConstants.turnPID.calculate(turnDiff); //calculates amt to turn
        double leftDiff = Util.deadband(LimelightConstants.horizontalOffset - vision.getHorizontalOffset(), LimelightConstants.epsilonHorizontal); //how far sidewyas to move across fied
        double left = LimelightConstants.horizontalPID.calculate(leftDiff);// ''
        double forwardDiff = Util.deadband( distance - LimelightConstants.desiredDistance - vision.getHorizontalDistance(), LimelightConstants.epsilonForward); //how far forward to go
        //System.out.println("turn diff, " + turnDiff + " left diff, " + leftDiff + " forward diff, " + forwardDiff);
        double forward = LimelightConstants.forwardPID.calculate(forwardDiff);
        if(!vision.isTargetFound()) {
            leftDiff = 0.0;
            forwardDiff = 0.0;
            left = 0.0;
            forward = 0.0;
        } else if(Math.abs(forwardDiff) < 0.2) {
            shooter.setup(TargetHeights.heights[height]);
        }
        drivetrain.drive(forward , -left , -turn, false, true);

        if(turnDiff == 0.0 && leftDiff == 0.0 && forwardDiff == 0.0) {
            lights.setColor(0, 255, 0);
            if(drivetrain.getAverageSpeed() <= 0.15) {
                isFinished = true;
            }
        } else {
            lights.setColor(255, 0, 0);
        }
    }
    public boolean isFinished() {
        return isFinished || (time != 0.0 && timer.get() >= time);
    }
    public void end(boolean interrupted) {
        System.out.println("done aligning");
        drivetrain.stop();
        vision.off();
    }
}