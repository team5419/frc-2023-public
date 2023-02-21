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
    public AutoBalance(Swerve drivetrain, Lights lights) {
        this.drivetrain = drivetrain;
        this.targetYaw = 0.0;
        this.lights = lights;
        hasShiftedBack = 0;
        this.timer = new Timer();
        addRequirements(drivetrain);
    }
    public void initialize() {
        hasShiftedBack = -2;
        this.targetYaw = Math.round(drivetrain.angle() / 180.0) * 180.0;
        timer.reset();
        timer.start();
        lights.rainbow();
    }
    public void execute() { 
        
        double yawDiff = Util.deadband(targetYaw - drivetrain.angle(), SwerveDriveConstants.epsilonYawBalance); //how far to turn (rotate) into correct postion
        double pitchDiff = -Util.deadband(drivetrain.anglePitch(), SwerveDriveConstants.epsilonBalance);  //how far to get balanced
        double yawChange = -SwerveDriveConstants.yawBalanceController.calculate(yawDiff);
        double pitchChange = 0.0;
        if(hasShiftedBack == -2) {
            if(pitchDiff > -11) {
                hasShiftedBack = -1;
            }
            pitchChange = -0.5;
        }
        if(hasShiftedBack == -1) {
            if(pitchDiff < -13) {
                hasShiftedBack = 0;
            }
            pitchChange = -0.5;
        }
        if(hasShiftedBack == 0) {
            if(pitchDiff > -11) {
                hasShiftedBack = 3;
            }
            pitchChange = -0.5;
        } 
        if(hasShiftedBack == 3) {
            pitchChange = Math.signum(pitchDiff) * 0.1;
        }
        System.out.println(hasShiftedBack);
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