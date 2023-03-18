package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Swerve;
import frc.robot.Util;
import frc.robot.Constants.SwerveDriveConstants;
public class AutoBalance extends CommandBase {
    private Swerve drivetrain;
    private Lights lights;
    private double targetYaw;
    private Timer timer;
    private int hasShiftedBack;
    private boolean facingForward;
    public AutoBalance(Swerve drivetrain, Lights lights) {
        this.drivetrain = drivetrain;
        this.targetYaw = 0.0;
        this.lights = lights;
        this.facingForward = false;
        hasShiftedBack = 0;
        this.timer = new Timer();
        addRequirements(drivetrain);
    }
    public void initialize() {
        hasShiftedBack = -1;
        int yawInt = (int)Math.round(drivetrain.angle() / 180.0);
        facingForward = (yawInt % 2) == 0;
        targetYaw = yawInt * 180.0;
        timer.reset();
        timer.start();
        lights.rainbow();
    }
    public void execute() { 
        
        double yawDiff = Util.deadband(targetYaw - drivetrain.angle(), SwerveDriveConstants.epsilonYawBalance); //how far to turn (rotate) into correct postion
        double pitchDiff = -Util.deadband(drivetrain.anglePitch(), SwerveDriveConstants.epsilonBalance);  //how far to get balanced
        double yawChange = -SwerveDriveConstants.yawBalanceController.calculate(yawDiff);
        double pitchChange = 0.0;
        // if(hasShiftedBack == -2) {
        //     if(pitchDiff > -9) {
        //         hasShiftedBack = -1;
        //     }
        //     pitchChange = -0.5;
        // }
        if(hasShiftedBack == -1) { // 9 and 14 degrees
            if(Math.abs(pitchDiff) > 12.5) {
                hasShiftedBack = 0;
            }
            pitchChange = 0.5 * Math.signum(pitchDiff);
        }
        if(hasShiftedBack == 0) {
            if(Math.abs(pitchDiff) < 11.5) {
                hasShiftedBack = 3;
            }
            pitchChange = 0.5 * Math.signum(pitchDiff);
        } 
        if(hasShiftedBack == 3) {
            if(Math.abs(pitchDiff) < 2.0) {
                pitchChange = Math.signum(pitchDiff) * 0.075;
            } else {
                pitchChange = pitchDiff * 0.012;
            }
        }
        if(facingForward) {
            pitchChange *= -1.0;
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