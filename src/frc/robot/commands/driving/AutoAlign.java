package frc.robot.commands.driving; // auto align using only limelight 
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.TargetHeights;
import frc.robot.subsystems.Coner;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
public class AutoAlign extends CommandBase {
    private Swerve drivetrain;
    private Vision vision;
    private double distance;
    private Coner shooter;
    private int height;
    private String convertedHeight;
    private Lights lights;
    private boolean isFinished;
    private Timer timer;
    private double time;
    private boolean secondPhase;
    public AutoAlign(Swerve drivetrain, Coner shooter, Vision vision, double distance, int height, Lights lights, double timeLimit) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.distance = distance;
        this.time = timeLimit;
        this.shooter = shooter;
        this.height = height;
        this.convertedHeight = TargetHeights.heights[this.height];
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
        shooter.elevatorOut(convertedHeight);
        
        vision.setPipelineToHigh(convertedHeight == TargetHeights.HIGH && !drivetrain.alwaysUseMid);
    }
    public void execute() {
        double theta = drivetrain.angle();
        double target = Math.round((theta - LimelightConstants.desiredAngle) / 360.0) * 360.0 + LimelightConstants.desiredAngle;
        double turnDiff = target - drivetrain.angle(); //calculates how many degrees to turn
        double turn = LimelightConstants.turnPID.calculate(turnDiff); //calculates amt to turn

        // limelight frame
        double rawDist = vision.getHorizontalDistance();
        double rawOffset = vision.getLinearHorizontalOffset(rawDist);
        // robot x-y frame
        // double robotDist = vision.getRobotRelativeHorizontalDistance(rawDist, rawOffset); // rawDist;
        // double robotOffset = vision.getRobotRelativeHorizontalOffset(rawDist, rawOffset); // rawOffset;
        // if 15 deg lineup
        double robotDist = rawDist;
        double robotOffset = rawOffset;

        double forwardDiff = distance - LimelightConstants.desiredDistance - robotDist; //how far forward to go

        double forward = LimelightConstants.forwardPID.calculate(forwardDiff);
        if (convertedHeight == TargetHeights.HIGH)
            forward = LimelightConstants.forawrdPIDHigh.calculate(forwardDiff);// ''

        double horiOffset = LimelightConstants.linHorizontalOffset;
        if(convertedHeight == TargetHeights.HIGH){
            horiOffset = LimelightConstants.linHorizontalOffsetHigh;
        }
        double leftDiff = horiOffset - robotOffset;

        double left = LimelightConstants.linHorizontalPID.calculate(leftDiff);// ''
        if (convertedHeight == TargetHeights.HIGH)
            left = LimelightConstants.linHorizontalPIDHigh.calculate(leftDiff);// ''


        double forward85 = forward * LimelightConstants.cosYaw - left * LimelightConstants.sinYaw;
        double left15 = forward * LimelightConstants.sinYaw + left * LimelightConstants.cosYaw;

        forward = forward85;
        left = left15;
        
        
        boolean found = vision.isTargetFound();
        if(found) {
            if(!secondPhase) {
                drivetrain.drive(forward ,-left , -turn, false, true);
            }
            if(Math.abs(forwardDiff) < 0.25) {
                shooter.setup(convertedHeight);
            }
        }
        if(secondPhase) {
            //shooter.setup(TargetHeights.heights[height]);
            drivetrain.stop();
        }
        double epsilonMultiplier = convertedHeight == TargetHeights.HIGH ? 0.5 : 1.0; // first number is high multiplier, second number is everything else multiplier
        if(Math.abs(turnDiff) <= LimelightConstants.epsilonTurn * epsilonMultiplier && Math.abs(leftDiff) <= LimelightConstants.epsilonLinHorizontal * epsilonMultiplier && Math.abs(forwardDiff) <= LimelightConstants.epsilonForward * epsilonMultiplier && found) {
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
        if(time != 0.0 && timer.get() >= time) {
            return true;
        }
        if(secondPhase) {
            return timer.get() >= 0.3;
        }
        if(isFinished) {
            if(convertedHeight == TargetHeights.LOW) {
                return true;
            }
            secondPhase = true;
            timer.reset();
            timer.start();
            drivetrain.stop();
        }
        return false;
    }
    public void end(boolean interrupted) {
        drivetrain.stop();
        vision.off();
        timer.stop();
    }
}