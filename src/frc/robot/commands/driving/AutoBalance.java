package frc.robot.commands.driving;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Swerve;
import frc.robot.Util;
import frc.robot.Constants.SwerveDriveConstants;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
public class AutoBalance extends CommandBase {
    private Swerve drivetrain;
    private Lights lights;
    private double targetYaw;
    private Timer timer;
    private int hasShiftedBack;
    private int turnToTag;
    private Vision vision;
    public AutoBalance(Swerve drivetrain, Lights lights, Vision vision, int turnToTag) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.targetYaw = 0.0;
        this.lights = lights;
        hasShiftedBack = 0;
        this.timer = new Timer();
        this.turnToTag = turnToTag;
        addRequirements(drivetrain);
    }
    public void initialize() {
        hasShiftedBack = -1;
        int yawInt = (int)Math.round(drivetrain.angle() / 180.0);
        targetYaw = yawInt * 180.0;
        timer.reset();
        timer.start();
        lights.rainbow();
    }
    public void execute() { 
        double pitchDiff = -Util.deadband(drivetrain.anglePitch(), SwerveDriveConstants.epsilonBalance);
        double yawDiff = 0.0;
        if(turnToTag != -1) {
            yawDiff = vision.getHorizontalToTarget(vision.team() == Alliance.Blue ? (9 - turnToTag) : turnToTag);
        }
        if(turnToTag == -1 || !vision.seesTag || Math.abs(pitchDiff) > 5) {
            yawDiff = Util.deadband(targetYaw - drivetrain.angle(), SwerveDriveConstants.epsilonYawBalance);
        }
          //how far to get balanced
        double yawChange = SwerveDriveConstants.pTheta * (Math.PI / 180.0) * yawDiff;
        double pitchChange = 0.0;
        if(hasShiftedBack == -1) { // 9 and 14 degrees
            if(Math.abs(pitchDiff) > 13.0) { // 12.5, 10.0 ours
                hasShiftedBack = 0;
            }
            pitchChange = 1.0 * Math.signum(pitchDiff); // 3.0
        }
        if(hasShiftedBack == 0) {
            if(Math.abs(pitchDiff) < 12.5) { // 11.5, 9.5 ours
                hasShiftedBack = 3;
            }
            pitchChange = 0.5 * Math.signum(pitchDiff); // 0.5
        } 
        if(hasShiftedBack == 3) {
            if(Math.abs(pitchDiff) < 2.0) {
                pitchChange = Math.signum(pitchDiff) * 0.11;
            } else {
                pitchChange = pitchDiff * 0.023;
            }
        }
        drivetrain.drive(pitchChange, 0.0, yawChange, false, true);
    }
    public boolean isFinished() {
        return false;
    }
    public void end(boolean interrupted) {
        drivetrain.stop();
        timer.stop();
        lights.off(drivetrain);
    }
}