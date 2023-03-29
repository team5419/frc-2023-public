package frc.robot.commands.driving;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Swerve;
import frc.robot.Util;
import frc.robot.Constants.SwerveDriveConstants;
public class TeleopBalance extends CommandBase {
    private Swerve drivetrain;
    private Lights lights;
    private double targetYaw;
    private XboxController controller;
    public TeleopBalance(Swerve drivetrain, Lights lights, XboxController controller) {
        this.drivetrain = drivetrain;
        this.targetYaw = 0.0;
        this.lights = lights;
        this.controller = controller;
        addRequirements(drivetrain);
    }
    public void initialize() {
        this.targetYaw = Math.round(drivetrain.angle() / 180.0) * 180.0;
        lights.rainbow();
    }
    public void execute() { 
        
        double yawDiff = Util.deadband(targetYaw - drivetrain.angle(), SwerveDriveConstants.epsilonYawBalance); //how far to turn (rotate) into correct postion
        double pitchDiff = -Util.deadband(drivetrain.anglePitch(), SwerveDriveConstants.epsilonBalance);  //how far to get balanced
        double yawChange = -SwerveDriveConstants.yawBalanceController.calculate(yawDiff);
        double pitchChange = 0.0;
        if(Math.abs(yawDiff) < 10.0) {
            if(Math.abs(pitchDiff) < 2.0) {
                pitchChange = Math.signum(pitchDiff) * 0.075;
            } else {
                pitchChange = pitchDiff * 0.025;
            }
        }
        double leftRight = Util.deadband(-controller.getLeftX(), SwerveDriveConstants.controllerDeadband) * SwerveDriveConstants.speedMultiplier;
        if(targetYaw % 180.0 == 0) {
            pitchChange *= -1.0;
            leftRight *= 1.0;
        }
        drivetrain.drive(pitchChange, leftRight, yawChange, false, true);
    }
    public boolean isFinished() {
        return false;
    }
    public void end(boolean interrupted) {
        drivetrain.stop();
        lights.off(drivetrain);
    }
}