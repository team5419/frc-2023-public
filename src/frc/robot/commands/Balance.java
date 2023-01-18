package frc.robot.commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants.Drive;
import frc.robot.Util;
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
        double yawDiff = targetYaw - drivetrain.angle();
        double pitchDiff = -drivetrain.anglePitch();
        double yawChange = -Drive.yawBalanceController.calculate(yawDiff);
        double pitchChange = -Drive.balanceController.calculate(pitchDiff);
        drivetrain.drive(pitchChange, Util.deadband(controller.getLeftX(), Drive.controllerDeadband) * Drive.speedMultiplier, yawChange, true, true);
        shouldFinish = Math.abs(yawDiff) < Drive.epsilonYawBalance && Math.abs(pitchDiff) < Drive.epsilonBalance && drivetrain.getAverageSpeed() < 0.1;
    }
    public boolean isFinished() {
        return shouldFinish;
    }
    public void end(boolean interrupted) {
        drivetrain.stop();
        // drivetrain.brake(); // maybe?
    }
}