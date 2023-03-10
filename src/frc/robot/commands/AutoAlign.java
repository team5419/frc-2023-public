package frc.robot.commands; // auto align using only limelight 
import frc.robot.Util;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimelightConstants;
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
    private boolean secondPhase;
    public AutoAlign(Swerve drivetrain, GenericShootIntake shooter, Vision vision, double distance, int height, Lights lights, double timeLimit) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.distance = distance;
        this.time = timeLimit;
        this.shooter = shooter;
        this.height = height;
        this.lights = lights;
        this.secondPhase = false;
        isFinished = false;
        addRequirements(drivetrain, shooter.subsystem());
        this.timer = new Timer();
    }
    public void initialize() {
        secondPhase = false;
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
        double rawDist = vision.getHorizontalDistance();
        double forwardDiff = Util.deadband( distance - LimelightConstants.desiredDistance - rawDist, LimelightConstants.epsilonForward); //how far forward to go
        //System.out.println("turn diff, " + turnDiff + " left diff, " + leftDiff + " forward diff, " + forwardDiff);
        double forward = LimelightConstants.forwardPID.calculate(forwardDiff);
        //double leftDiff = Util.deadband(LimelightConstants.horizontalOffset - vision.getHorizontalOffset(), LimelightConstants.epsilonHorizontal); //how far sidewyas to move across fied
        double leftDiff = Util.deadband(LimelightConstants.linHorizontalOffset - vision.getLinearHorizontalOffset(rawDist), LimelightConstants.epsilonLinHorizontal);
        double left =  0.0;
        //if(Math.abs(leftDiff) >= 2.5) {
            left = LimelightConstants.linHorizontalPID.calculate(leftDiff);// ''
        /*{  else {
            left = LimelightConstants.closeConstant * -Math.signum(leftDiff);
        }*/
        
        
        boolean found = vision.isTargetFound();
        if(found && Math.abs(forwardDiff) < 0.2 && !secondPhase) {
            shooter.setup(TargetHeights.heights[height]);
        }
        if(found && !secondPhase) {
            drivetrain.drive(forward , -left , -turn, false, true);
        }
        if(secondPhase) {
            drivetrain.stop();
        }
        if(turnDiff == 0.0 && leftDiff == 0.0 && forwardDiff == 0.0 && found) {
            lights.setColor(0, 255, 0);
            if(drivetrain.getAverageSpeed() <= 0.1) {
                isFinished = true;
            }
        } else {
            lights.setColor(255, 0, 0);
            if(secondPhase && found) {
                isFinished = false;
                secondPhase = false;
            }
        }
    }
    public boolean isFinished() {
        if(secondPhase) {
            return timer.get() >= 0.25;
        }
        boolean fin= isFinished || (time != 0.0 && timer.get() >= time);
        //return fin;
        if(fin) {
            secondPhase = true;
            timer.reset();
            timer.start();
            drivetrain.stop();
        }
        return false;
    }
    public void end(boolean interrupted) {
        System.out.println("done aligning");
        drivetrain.stop();
        vision.off();
        timer.stop();
    }
}