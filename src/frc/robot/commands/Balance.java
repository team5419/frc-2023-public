package frc.robot.commands;
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
    public Balance(Swerve drivetrain, XboxController controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        this.targetYaw = Math.round(drivetrain.angle() / 180.0) * 180.0;
        shouldFinish = false;
        addRequirements(drivetrain);
    }
    public void initialize() {
        
    }
    public void execute() { 
        double yawDiff = targetYaw - drivetrain.angle(); //how far to turn (rotate) into correct postion
        double pitchDiff = -drivetrain.anglePitch();  //how far to get balanced
        double yawChange = -SwerveDriveConstants.yawBalanceController.calculate(yawDiff);
        double pitchChange = -SwerveDriveConstants.balanceController.calculate(pitchDiff); //returns how far needed to drive in order to balance
        drivetrain.drive(pitchChange, Util.deadband(controller.getLeftX(), SwerveDriveConstants.controllerDeadband) * SwerveDriveConstants.speedMultiplier, yawChange, true, true);
        shouldFinish = Math.abs(yawDiff) < SwerveDriveConstants.epsilonYawBalance && Math.abs(pitchDiff) < SwerveDriveConstants.epsilonBalance && drivetrain.getAverageSpeed() < 0.1;
    }
    public boolean isFinished() {
        return shouldFinish;
    }
    public void end(boolean interrupted) {
        drivetrain.stop();
        // drivetrain.brake(); // maybe?
    }
}