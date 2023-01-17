package frc.robot.commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.Util;
import frc.robot.Constants.Drive;

public class SwerveDrive extends CommandBase {
    XboxController driver;
    Swerve drivetrain;

    public SwerveDrive(Swerve drivetrain, XboxController _driver){
        driver = _driver;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    public void initialize() {

    }
    public void execute() {
        drivetrain.drive(
            Util.deadband(-driver.getLeftY(), Drive.controllerDeadband) * Drive.speedMultiplier,
            Util.deadband(-driver.getLeftX(), Drive.controllerDeadband) * Drive.speedMultiplier, 
            Util.deadband(-driver.getRightX(), Drive.controllerDeadband) * Drive.turnMultiplier, true, false);
    }
    public boolean isFinished() {
        return false;
    }
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
