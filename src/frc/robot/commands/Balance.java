package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.Util;
import frc.robot.Constants.SwerveDriveConstants;
public class Balance extends CommandBase {
    private Swerve drivetrain;
    private boolean shouldFinish;
    private XboxController controller;
    private double targetYaw;
    private Timer timer;
    private int hasShiftedBack;
    public Balance(Swerve drivetrain, XboxController controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        this.targetYaw = 0.0;
        hasShiftedBack = 0;
        shouldFinish = false;
        this.timer = new Timer();
        addRequirements(drivetrain);
    }
    public void initialize() {
        hasShiftedBack = -2;
        this.targetYaw = Math.round(drivetrain.angle() / 180.0) * 180.0;
        timer.reset();
        timer.start();
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
        } /*if(hasShiftedBack == 1) {
            if(pitchDiff > 7.0) {
                hasShiftedBack = 2;
            }
            pitchChange = 0.3;
        } if(hasShiftedBack == 2) {
            if(pitchDiff < 6.0) {
                hasShiftedBack = 3;
            }
            pitchChange = 0.3;
        }*/ if(hasShiftedBack == 3) {
            pitchChange = Math.signum(pitchDiff) * 0.1;
        }
        System.out.println(hasShiftedBack);
        drivetrain.drive(pitchChange, Util.deadband(controller == null ? 0.0 : controller.getLeftX(), SwerveDriveConstants.controllerDeadband) * SwerveDriveConstants.speedMultiplier, yawChange, false, true);
        //shouldFinish = yawDiff == 0.0 && pitchDiff == 0.0 && drivetrain.getAverageSpeed() < 0.1;
    }
    public boolean isFinished() {
        return /*shouldFinish ||*/controller != null &&  Math.abs(controller.getRightX()) > SwerveDriveConstants.controllerDeadband;
    }
    public void end(boolean interrupted) {
        drivetrain.stop();
        timer.stop();
        // drivetrain.brake(); // maybe?
    }
}