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
    public Balance(Swerve drivetrain, XboxController controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        shouldFinish = false;
        addRequirements(drivetrain);
    }
    public void initialize() {
        
    }
    public void execute() { 
        double yawChange = Drive.yawBalanceController.calculate(drivetrain.angle());
        double pitchChange = Drive.balanceController.calculate(drivetrain.anglePitch());
        drivetrain.drive(pitchChange, Util.deadband(controller.getLeftX(), Drive.controllerDeadband) * Drive.speedMultiplier, yawChange);
        shouldFinish = Math.abs(yawChange) < Drive.epsilonYawBalance && Math.abs(pitchChange) < Drive.epsilonBalance;
    }
    public boolean isFinished() {
        return shouldFinish;
    }
    public void end(boolean interrupted) {
        drivetrain.drive(0.0, 0.0, 0.0);
    }
}